"""Local UDP bridge. Only received, delayed samples enter CaptureSession."""
import argparse
import asyncio
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys

from .hocd_tirex_adapter import ExternalTiRexModel, RESOURCE_ROOT, RUN_ROOT, warmup_model
from mytirex.inference.delivery import ComputeLatencyEstimator
from mytirex.signals import SignalSchema
from mytirex.streaming.runtime import StreamConfig, StreamingRuntime, sleep_until
from mytirex.streaming.source import SignalSource
from mytirex.timing.clock import ClockConfig, observation_timing
from mytirex.timing.receiver import TimingConfig
from mytirex.timing.transport import CaptureSession, decode, encode, monotonic_us, timestamp_event


class DelayedPackets:
    """Delay signal datagrams only; four-timestamp responses bypass the queue."""
    def __init__(self, transport, delay_us):
        self.transport, self.delay_us = transport, delay_us
        self.loop = asyncio.get_running_loop()
        self.handles = set()

    def sendto(self, payload, address):
        message = decode(payload)
        if message['kind'] != 'packet':
            self.transport.sendto(payload, address)
            return
        due = message['source_time_us'] + self.delay_us
        handle = None

        def release():
            self.handles.discard(handle)
            self.transport.sendto(payload, address)

        handle = self.loop.call_later(max(0, due - monotonic_us()) / 1e6, release)
        self.handles.add(handle)

    def close(self):
        for handle in self.handles:
            handle.cancel()
        self.handles.clear()
        self.transport.close()


class RawSteering(asyncio.DatagramProtocol):
    def __init__(self, trial, log):
        self.trial, self.log = trial, log
        self.samples = deque(maxlen=2048)
        self.counts = Counter()
        self.last_seq = self.last_time = -1

    def datagram_received(self, data, address):
        received = monotonic_us()
        try:
            msg = json.loads(data)
            if (address[0] != '127.0.0.1' or msg['kind'] != 'raw' or msg['trial_id'] != self.trial
                    or type(msg['sequence']) is not int or msg['sequence'] <= self.last_seq
                    or type(msg['native_source_time_us']) is not int
                    or not self.last_time < msg['native_source_time_us'] <= received
                    or not math.isfinite(msg['steer_h']) or abs(msg['steer_h']) > 3.1416):
                raise ValueError('Invalid/out-of-order raw steering')
            self.last_seq, self.last_time = msg['sequence'], msg['native_source_time_us']
            self.samples.append((self.last_time, float(msg['steer_h'])))
            self.log.write(json.dumps({**msg, 'bridge_receive_us': received}) + '\n')
            self.counts['raw_samples'] += 1
        except (KeyError, TypeError, ValueError):
            self.counts['rejected'] += 1

    def at_grid(self, grid):
        # Keep the last sample before the grid plus any newer pending samples.
        eligible = [s for s in self.samples if s[0] <= grid]
        if not eligible or grid - eligible[-1][0] > 50_000:
            return None
        return eligible[-1]


class ReceiverDatagrams(asyncio.DatagramProtocol):
    def __init__(self, runtime, log):
        self.runtime, self.log = runtime, log
        self.pending = {}
        self.counts = Counter()
        self.mapping_revision = 0
        self.latest_native = None

    def datagram_received(self, data, address):
        received = monotonic_us()
        try:
            msg = decode(data)
            event = timestamp_event(msg, received, self.runtime.session.receiver_clock_id, self.pending)
            self.runtime.process(event)
            self.log.write(json.dumps(event, allow_nan=False) + '\n')
            if msg['kind'] == 'sync_response':
                self.pending.pop(msg['request_id'], None)
                self.mapping_revision += 1
            elif msg['kind'] == 'packet':
                if self.latest_native is None or msg['sequence'] > self.latest_native[0]:
                    self.latest_native = (msg['sequence'], msg['native_source_time_us'])
            self.counts[msg['kind']] += 1
        except (KeyError, TypeError, ValueError, OSError) as exc:
            self.counts['rejected'] += 1
            self.log.write(json.dumps({'kind': 'rejected', 'error': str(exc), 'received_at_us': received}) + '\n')

    async def synchronize(self, transport, finish):
        request_id = 0
        due = monotonic_us()
        while due < finish:
            await sleep_until(due)
            sent = monotonic_us()
            self.pending = {k: v for k, v in self.pending.items() if sent - v < 5_000_000}
            self.pending[request_id] = sent
            transport.sendto(encode({'kind': 'sync_request',
                'receiver_clock_id': self.runtime.session.receiver_clock_id,
                'request_id': request_id, 'receiver_send_us': sent}))
            request_id += 1
            due = max(due + 250_000, sent + 1)


async def serve(args):
    output = Path(args.run_dir).resolve()
    if not output.is_relative_to(RUN_ROOT / 'runs'):
        raise ValueError('Run output must be in cooperative_driving/runs')
    output.mkdir(parents=True, exist_ok=True)
    schema_config = json.loads((RUN_ROOT / 'configs/hocd_signals.json').read_text())
    period = schema_config.pop('sample_period_us')
    schema = SignalSchema(**schema_config)
    delay = 0 if args.condition == 'direct' else 300_000
    session = CaptureSession('hocd-receiver-' + args.trial_id, ClockConfig(), TimingConfig(),
        sample_period_us=period, signal_schema=schema, context_length=256, minimum_observations=256)
    # One 25 ms grid margin rounds UP the required extra 10 ms control period.
    config = StreamConfig(margin_steps=1, warmup_requests=0)
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stop.set)
    model = None
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='current-tirex')
    transports, tasks = [], []
    runtime = protocol = raw = source = None
    result = {'status': 'starting', 'trial_id': args.trial_id, 'condition': args.condition,
              'delay_us': delay, 'stream': asdict(config), 'control_margin_us': 10_000}
    try:
        estimator = ComputeLatencyEstimator()
        if args.condition == 'tirex':
            print('Loading current TiRex and warming kernels...', flush=True)
            model = ExternalTiRexModel()
            await loop.run_in_executor(executor, model.load)
            warmup = await loop.run_in_executor(executor, warmup_model, model)
            for measured in warmup[1:]:
                estimator.update(measured * 1e6)
            result['pre_drive_warmup_s'] = warmup
        with (output / 'bridge-raw.jsonl').open('w', buffering=1) as raw_log, \
             (output / 'bridge-events.jsonl').open('w', buffering=1) as events, \
             (output / 'bridge-publications.jsonl').open('w', buffering=1) as publications, \
             (output / 'bridge-inferences.jsonl').open('w', buffering=1) as inferences:
            reply, _ = await loop.create_datagram_endpoint(asyncio.DatagramProtocol,
                remote_addr=('127.0.0.1', 5006))
            transports.append(reply)
            tx_sequence = 0
            last_trajectory_key = None

            def send(kind, payload):
                nonlocal tx_sequence
                msg = {'version': 1, 'kind': kind, 'trial_id': args.trial_id,
                       'sequence': tx_sequence, 'sent_at_us': monotonic_us(), **payload}
                data = json.dumps(msg, separators=(',', ':'), allow_nan=False).encode()
                if len(data) > 8192:
                    raise ValueError('Trajectory exceeds one UDP datagram')
                reply.sendto(data)
                tx_sequence += 1

            def trajectory(force=False):
                nonlocal last_trajectory_key
                if runtime.cache is None:
                    return
                cache = runtime.cache
                work, forecast = cache.work, cache.work.trajectory
                key = (work.request_id, protocol.mapping_revision)
                if not force and key == last_trajectory_key:
                    return
                now = monotonic_us()
                mapping = session.clock.snapshot(now)
                timing = observation_timing(forecast.source_time_us, None, now, period, mapping,
                    source_error_us=session.timing_config.source_error_us,
                    query_error_us=session.timing_config.query_error_us,
                    maximum_uncertainty_us=session.timing_config.maximum_uncertainty_us)
                mapped = None
                if mapping.valid:
                    mapped = now - mapping.age(forecast.source_time_us, now).estimate_us
                send('trajectory', {'forecast_id': work.request_id,
                    's_f': forecast.source_time_us, 'anchor_value': work.view.last_values['steer_h'],
                    'sample_period_us': period, 'median': forecast._median()[0].tolist(),
                    't_f': mapped, 'r': mapping.rate,
                    'valid': bool(timing.valid and forecast.status == 'ok'),
                    'mapping_status': timing.reason, 'mapping_snapshot_us': now,
                    'compute_us': work.compute_us, 'forecast_completed_at_us': work.completed_at_us})
                last_trajectory_key = key

            def publish(row):
                publications.write(json.dumps(row, allow_nan=False) + '\n')
                source_time = row['last_source_time_us']
                now = monotonic_us()
                mapped = None
                rate = 1.
                if session.clock is not None:
                    mapping = session.clock.snapshot(now)
                    rate = mapping.rate
                    if source_time is not None and mapping.valid:
                        mapped = now - mapping.age(source_time, now).estimate_us
                send('observation', {'last_observation': row['last_observation'].get('steer_h'),
                    'source_time_us': source_time,
                    'native_source_time_us': protocol.latest_native[1] if protocol.latest_native else None,
                    'mapped_source_time_us': mapped, 'r': rate,
                    'aoi_us': row['aoi_us'], 'timing_valid': row['timing_valid'],
                    'context_ready': row['context_ready'], 'condition': args.condition})
                trajectory()

            def inference(row):
                inferences.write(json.dumps(row, allow_nan=False) + '\n')
                if row['installed']:
                    trajectory(force=True)

            runtime = StreamingRuntime(session, config, model=model, executor=executor,
                estimator=estimator, on_publish=publish, on_inference=inference)
            source = SignalSource(schema, source_clock_id='hocd-source-' + args.trial_id)
            tx, _ = await loop.create_datagram_endpoint(lambda: source, local_addr=('127.0.0.1', 5005))
            delayed = DelayedPackets(tx, delay)
            source.transport = delayed
            transports.append(delayed)
            raw = RawSteering(args.trial_id, raw_log)
            rx, _ = await loop.create_datagram_endpoint(lambda: raw, local_addr=('127.0.0.1', 5004))
            transports.append(rx)
            protocol = ReceiverDatagrams(runtime, events)
            receiver, _ = await loop.create_datagram_endpoint(lambda: protocol, remote_addr=('127.0.0.1', 5005))
            transports.append(receiver)
            start = monotonic_us()
            finish = start + int(args.duration_s * 1e6)

            async def sample_grid():
                grid = ((monotonic_us() + period - 1) // period) * period
                origin = grid
                while grid < finish:
                    await sleep_until(grid)
                    now = monotonic_us()
                    # Skip elapsed source ticks; do not manufacture catch-up packets.
                    if now - grid >= period:
                        grid += (now - grid) // period * period
                    sample = raw.at_grid(grid)
                    if sample is not None:
                        native, value = sample
                        source.publish({'steer_h': value}, sequence=(grid - origin) // period,
                            source_time_us=grid, native_source_time_us=native)
                    grid += period

            tasks = [asyncio.create_task(sample_grid()),
                     asyncio.create_task(protocol.synchronize(receiver, finish)),
                     asyncio.create_task(runtime.run(finish - start, start_us=start))]
            ready = {**result, 'status': 'ready', 'ready_at_us': monotonic_us(),
                     'model_warmed': model is not None}
            (output / 'bridge-ready.json').write_text(json.dumps(ready, indent=2) + '\n')
            print(f'Bridge ready: {args.trial_id} {args.condition}; 5004 → 5005 → 5006', flush=True)
            stop_task = asyncio.create_task(stop.wait())
            done, _ = await asyncio.wait([*tasks, stop_task], return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                if task is not stop_task:
                    task.result()
            stop_task.cancel()
            result['status'] = 'stopped' if stop.is_set() else 'finished'
    except BaseException as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if runtime is not None:
            runtime._running = False
            if hasattr(runtime, 'finish_us'):
                runtime.finish_us = min(runtime.finish_us, monotonic_us())
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        for transport in reversed(transports):
            transport.close()
        executor.shutdown(wait=True)
        result.update(runtime_counts=dict(runtime.counts) if runtime else {},
            input_counts=dict(raw.counts) if raw else {}, receiver_counts=dict(session.counts),
            protocol_counts=dict(protocol.counts) if protocol else {},
            source_counts=dict(source.counts) if source else {},
            final_compute_estimate_us=runtime.estimator.estimate_us if runtime else None,
            model=model.metadata() if model is not None and model.model is not None else None)
        (output / 'bridge-summary.json').write_text(json.dumps(result, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial-id', required=True)
    parser.add_argument('--condition', choices=('direct', 'hold', 'tirex'), required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--duration-s', type=float, default=180.)
    args = parser.parse_args()
    if not 1 <= args.duration_s <= 3600:
        parser.error('Duration must be in 1..3600 seconds')
    asyncio.run(serve(args))


if __name__ == '__main__':
    main()
