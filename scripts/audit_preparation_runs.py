"""Audit the recorded real bridge path, delay injection and applied fusion."""
import json
from pathlib import Path
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def main():
    report = {}
    for condition in ('direct', 'hold', 'tirex'):
        directory = ROOT / 'runs' / f'prep_{condition}_full_02'
        raw = pd.read_json(directory / 'bridge-raw.jsonl', lines=True)
        records = [json.loads(line) for line in (directory / 'bridge-events.jsonl').read_text().splitlines()]
        packets = [e for e in records if e['kind'] == 'packet']
        syncs = [e for e in records if e['kind'] == 'sync']
        native_values = dict(zip(raw.native_source_time_us, raw.steer_h))
        expected_delay = 0 if condition == 'direct' else 300_000
        delays = [p['received_at_us'] - p['source_time_us'] for p in packets]
        causal = all(p['native_source_time_us'] <= p['source_time_us'] <= p['send_time_us']
            <= p['received_at_us'] and p['native_source_time_us'] in native_values
            and abs(p['values']['steer_h'] - native_values[p['native_source_time_us']]) < 1e-12 for p in packets)
        regular = all((p['source_time_us'] - q['source_time_us']) ==
            (p['sequence'] - q['sequence']) * 25_000 for q, p in zip(packets, packets[1:]))
        ticks = pd.read_csv(directory / 'ticks.csv')
        active = ticks[(ticks.phase == 'score') & ticks.authority.notna()]
        expected_mix = active.authority * active.automatic_steer + (1 - active.authority) * active.human_steer_est
        fusion = bool(np.allclose(expected_mix, active.mixed_steer, rtol=0, atol=1e-12))
        actuator = bool(np.allclose(np.clip(active.mixed_steer, -1, 1), active.applied_steer, rtol=0, atol=1e-12))
        packet_values = {p['source_time_us']: p['values']['steer_h'] for p in packets}
        observations = all(abs(packet_values[int(t)] - value) < 1e-12
            for t, value in zip(active.last_source_time_us, active.last_observation))
        hold = condition == 'tirex' or bool(np.allclose(active.human_steer_est, active.last_observation, rtol=0, atol=1e-12))
        rtt = [(s['receiver_receive_us'] - s['receiver_send_us']) -
               (s['source_send_us'] - s['source_receive_us']) for s in syncs]
        frames = bool((active.actual_apply_frame == active.target_frame).all())
        passed = bool(causal and regular and min(delays) >= expected_delay and fusion and actuator and observations and hold and frames)
        report[condition] = {'passed': passed, 'causal_grid_values': causal, 'regular_source_grid': regular,
            'minimum_injected_data_age_ms': min(delays) / 1000,
            'median_injected_data_age_ms': float(np.median(delays) / 1000),
            'p95_sync_round_trip_ms': float(np.percentile(rtt, 95) / 1000),
            'fusion_uses_selected_estimate': fusion, 'actuator_conversion_consistent': actuator,
            'hold_values_are_received_packets': observations and hold, 'target_frames_match': frames}
    result = {'passed': all(v['passed'] for v in report.values()), 'conditions': report}
    (ROOT / 'logs/bridge-causality-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
