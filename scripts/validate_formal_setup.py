"""Explicitly marked scripted diagnostics, never a live/formal experiment."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
from zoneinfo import ZoneInfo

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge.experiment import MAPS, FORMAL_CONDITIONS


def audit(path):
    config = json.loads((path / 'config.json').read_text())
    summary = json.loads((path / 'summary.json').read_text())
    metrics = json.loads((path / 'metrics.json').read_text())
    ticks = pd.read_csv(path / 'ticks.csv')
    score = ticks[ticks.phase == 'score']
    trajectory = pd.read_csv(path / 'trajectory.csv')
    checks = {
        'scripted_not_formal': config['diagnostic_input'] and not config['formal'],
        'completed': summary['completed'] and summary['exit_reason'] == 'route_complete',
        'clean': not summary['collisions'] and not summary['cleanup_errors'] and not summary.get('artifact_error'),
        'duration': 30 <= summary['scoring_duration_s'] <= 45,
        'paced': .97 <= summary['sim_wall_ratio'] <= 1.03 and summary['p95_tick_interval_ms'] < 15,
        'no_frame_mismatch': summary['frame_mismatches'] == 0,
        'trajectory_complete': len(trajectory) == len(ticks) + 1 and metrics['trajectory_frame_gaps'] == 0,
        'live_camera': summary['camera_frames'] > 500 and summary['displayed_images'] > 500,
        'three_static_files': all((path / n).exists() for n in ('scenario.json', 'scene.json', 'scoring_reference.csv')),
        'delay_and_speed': config['delay_us'] == 300_000 and config['config']['controller']['target_speed_kmh'] == 30,
        'received_input_age': 300 <= metrics['mean_native_aoi_at_actual_tick_ms'] < 450,
        'no_saved_images': not list(path.glob('*.png')) and not list(path.glob('*.svg')),
    }
    if config['prediction']:
        checks['prediction_working'] = (metrics.get('inference_failures') == 0
            and metrics['trajectory_fraction'] > .95 and metrics['control_margin_covered'])
    if config['condition'] == 'manual_tirex':
        checks['no_automatic_steering'] = (np.allclose(score.authority, 0.)
            and np.allclose(score.automatic_steer, 0.)
            and np.allclose(score.mixed_steer, score.human_steer_est)
            and np.allclose(score.applied_steer, np.clip(score.human_steer_est, -1., 1.)))
    return {'directory': str(path), 'map': config['map'], 'condition': config['condition'],
        'passed': all(checks.values()), 'checks': {k: bool(v) for k, v in checks.items()},
        'scoring_duration_s': summary['scoring_duration_s'], 'lateral_rmse_m': metrics['lateral_rmse_m'],
        'mean_tick_interval_ms': metrics['mean_tick_interval_ms'],
        'trajectory_fraction': metrics['trajectory_fraction'],
        'fallback_fraction': metrics['fallback_fraction']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-diagnostics', action='store_true', help='Generate scripted input; never use the connected wheel')
    parser.add_argument('--prefix', default='prep_v2_acceptance')
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    if not args.run_diagnostics and not args.audit_only:
        parser.error('Select --run-diagnostics or --audit-only')
    records = []
    for town in MAPS:
        for condition in FORMAL_CONDITIONS:
            name = f'{args.prefix}_{town}_{condition}'
            path = ROOT / 'runs' / name
            if args.run_diagnostics:
                if path.exists():
                    parser.error(f'{path} already exists; choose a new --prefix')
                print(f'DIAGNOSTIC ONLY: {town} / {condition}', flush=True)
                with (ROOT / 'logs' / (name + '.log')).open('w') as log:
                    subprocess.run(['bash', str(ROOT / 'scripts/run_trial.sh'), '--map', town,
                        '--condition', condition, '--diagnostic-input', '--trial-id', name],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
            result = audit(path)
            records.append(result)
            print(json.dumps(result), flush=True)
            if not result['passed']:
                raise RuntimeError('Diagnostic audit failed; inspect the failed checks')
    import carla
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.)
    world = client.get_world()
    clean = not world.get_settings().synchronous_mode and not [a for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
    report = {'checked_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'passed': all(r['passed'] for r in records) and clean, 'trials': records, 'carla_idle': clean,
        'note': 'Program-generated steering only; these are not human/formal observations.'}
    (ROOT / 'logs/formal-preparation-audit.json').write_text(json.dumps(report, indent=2) + '\n')
    assert report['passed']
    print('All nine preparation diagnostics passed; no live wheel trials were launched.', flush=True)


if __name__ == '__main__':
    main()
