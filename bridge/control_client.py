"""Python 3.10 side: causal delayed Hold and fixed-anchor interpolation."""
from dataclasses import dataclass
import json
import math
import socket
from time import monotonic_ns


def monotonic_us():
    return monotonic_ns() // 1000


@dataclass(frozen=True)
class Selection:
    value: float
    mode: str
    reason: str
    source_time_us: int | None = None
    native_source_time_us: int | None = None
    hold_value: float | None = None
    aoi_us: float | None = None
    forecast_id: int | None = None
    forecast_source_time_us: int | None = None
    source_offset_us: float | None = None
    compute_us: int | None = None
    timing_valid: bool = False


class SteeringClient:
    def __init__(self, trial_id, condition, *, bind=('127.0.0.1', 5006), destination=('127.0.0.1', 5004)):
        self.trial_id, self.condition = trial_id, condition
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(bind)
        self.socket.setblocking(False)
        self.destination = destination
        self.raw_sequence = 0
        self.observation = self.trajectory = None
        self.sequences = {}
        self.rejected = 0

    def send_raw(self, steer, timestamp):
        if not math.isfinite(steer) or abs(steer) > 3.1416:
            raise ValueError('Invalid native steering sample')
        self.socket.sendto(json.dumps({'kind': 'raw', 'trial_id': self.trial_id,
            'sequence': self.raw_sequence, 'native_source_time_us': timestamp,
            'steer_h': float(steer)}, separators=(',', ':'), allow_nan=False).encode(), self.destination)
        self.raw_sequence += 1

    def ingest(self, msg):
        """Validate before replacing either cached message."""
        kind = msg.get('kind')
        if (msg.get('version') != 1 or msg.get('trial_id') != self.trial_id
                or kind not in ('observation', 'trajectory')
                or type(msg.get('sequence')) is not int
                or msg['sequence'] <= self.sequences.get(kind, -1)):
            raise ValueError('Stale/cross-trial message')
        if kind == 'observation':
            value = msg.get('last_observation')
            if value is not None:
                if not math.isfinite(value) or type(msg['source_time_us']) is not int:
                    raise ValueError('Invalid received observation')
                old = self.observation
                if old and old.get('source_time_us') is not None and msg['source_time_us'] < old['source_time_us']:
                    raise ValueError('Observation moved backwards')
            self.observation = dict(msg)
        else:
            values = msg['median']
            if (type(msg['forecast_id']) is not int or type(msg['s_f']) is not int
                    or msg['sample_period_us'] != 25_000 or not isinstance(values, list)
                    or not 1 <= len(values) <= 320 or not all(math.isfinite(v) for v in values)
                    or not math.isfinite(msg['anchor_value']) or not math.isfinite(msg['r']) or msg['r'] <= 0
                    or type(msg['valid']) is not bool
                    or msg['valid'] and (msg['t_f'] is None or not math.isfinite(msg['t_f']))):
                raise ValueError('Invalid trajectory')
            old = self.trajectory
            if old:
                if msg['forecast_id'] < old['forecast_id']:
                    raise ValueError('Older forecast')
                if msg['forecast_id'] == old['forecast_id'] and (msg['s_f'] != old['s_f']
                        or msg['anchor_value'] != old['anchor_value'] or values != old['median']):
                    raise ValueError('A mapping update must retain the original source anchor and trajectory')
            self.trajectory = dict(msg)
        self.sequences[kind] = msg['sequence']

    def drain(self):
        while True:
            try:
                data, address = self.socket.recvfrom(8193)
            except BlockingIOError:
                break
            try:
                if len(data) > 8192 or address[0] != '127.0.0.1':
                    raise ValueError('Invalid datagram')
                self.ingest(json.loads(data))
            except (KeyError, TypeError, ValueError):
                self.rejected += 1

    @property
    def context_ready(self):
        return bool(self.observation and self.observation.get('context_ready'))

    def select(self, t_use):
        """Select anew for the planned next physical step; never extrapolate."""
        obs = self.observation
        if obs is None or obs['last_observation'] is None:
            return Selection(0., 'unavailable', 'no_received_observation')
        mapped = obs.get('mapped_source_time_us')
        age = t_use - mapped if mapped is not None else None
        common = dict(source_time_us=obs['source_time_us'],
            native_source_time_us=obs.get('native_source_time_us'),
            hold_value=float(obs['last_observation']), aoi_us=age,
            timing_valid=bool(obs.get('timing_valid')))
        if self.condition != 'tirex':
            return Selection(float(obs['last_observation']), 'hold', 'condition_hold', **common)
        forecast = self.trajectory
        reason = 'awaiting_forecast'
        if forecast is not None:
            reason = 'invalid_mapping'
            if forecast['valid'] and obs.get('timing_valid'):
                offset = (t_use - forecast['t_f']) / forecast['r']
                horizon = len(forecast['median']) * forecast['sample_period_us']
                if 0 <= offset <= horizon:
                    position = offset / forecast['sample_period_us']
                    index = min(int(position), len(forecast['median']) - 1)
                    values = [forecast['anchor_value'], *forecast['median']]
                    weight = position - index
                    value = values[index] + weight * (values[index + 1] - values[index])
                    if math.isfinite(value):
                        return Selection(value, 'trajectory', 'ok', **common,
                            forecast_id=forecast['forecast_id'], forecast_source_time_us=forecast['s_f'],
                            source_offset_us=offset, compute_us=forecast.get('compute_us'))
                reason = 'trajectory_outside_coverage'
        return Selection(float(obs['last_observation']), 'hold', reason, **common,
            forecast_id=forecast.get('forecast_id') if forecast else None,
            forecast_source_time_us=forecast.get('s_f') if forecast else None,
            compute_us=forecast.get('compute_us') if forecast else None)

    def close(self):
        self.socket.close()
