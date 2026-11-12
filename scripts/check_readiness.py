"""Non-driving checks for the handoff immediately before T248P connection."""
from datetime import datetime
from importlib.metadata import version
import json
import os
from pathlib import Path
import socket
import re
import subprocess
import sys
from zoneinfo import ZoneInfo

import carla

ROOT = Path(__file__).resolve().parents[1]


def main():
    checks = {}
    expected = {'torch': '1.13.1+cpu', 'carla': '0.9.16', 'numpy': '1.24.4',
        'cvxopt': '1.3.2', 'gym': '0.26.2', 'matplotlib': '3.7.5', 'networkx': '3.1',
        'PyYAML': '6.0.2', 'omegaconf': '2.3.0', 'pandas': '2.0.3', 'seaborn': '0.13.2',
        'pygame': '2.6.1', 'tabulate': '0.9.0'}
    actual = {name: version(name) for name in expected}
    checks['hocd_environment'] = {'passed': actual == expected and sys.version_info[:2] == (3, 10),
                                  'python': sys.version, 'packages': actual}
    hocd = ROOT / 'vendor/HOCD'
    revision = subprocess.check_output(['git', '-C', str(hocd), 'rev-parse', 'HEAD'], text=True).strip()
    checks['hocd_base'] = {'passed': revision == '49b0189595f3dc525f4bc7194671742e203f7839', 'revision': revision}
    test_log = (ROOT / 'logs/unit-tests.log').read_text()
    match = re.search(r'Ran (\d+) tests', test_log)
    checks['regression_tests'] = {'passed': '\nOK\n' in test_log and match is not None and int(match[1]) >= 10,
                                  'test_count': int(match[1]) if match else 0}
    for port in (5004, 5005, 5006):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(('127.0.0.1', port))
            checks[f'udp_{port}_free'] = {'passed': True}
        except OSError as exc:
            checks[f'udp_{port}_free'] = {'passed': False, 'error': str(exc)}
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.)
    try:
        world = client.get_world()
        actors = [a.type_id for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
        settings = world.get_settings()
        checks['carla_idle'] = {'passed': client.get_server_version() == '0.9.16'
            and world.get_map().name.split('/')[-1] == 'Town04' and not actors and not settings.synchronous_mode,
            'server': client.get_server_version(), 'town': world.get_map().name,
            'remaining_vehicle_sensor_actors': actors, 'synchronous_mode': settings.synchronous_mode}
    except Exception as exc:
        checks['carla_idle'] = {'passed': False, 'error': str(exc)}
    warmup = json.loads((ROOT / 'logs/tirex-warmup.json').read_text())
    checks['current_model_warmup'] = {'passed': warmup['status'] == 'passed' and warmup['source'] == 'current'
        and warmup['source_directory'] == '/home/patton/MyTirex/vendor/tirex'
        and warmup['runtime_root'] == str(ROOT), 'warmup_s': warmup['warmup_s']}
    checks['hocd_config_copy'] = {'passed': (ROOT / 'configs/hocd.yaml').read_bytes() == (hocd / 'utils/config.yaml').read_bytes()}
    route = json.loads((ROOT / 'configs/route_town04.json').read_text())
    route_hash = __import__('hashlib').sha256((ROOT / 'configs/route_town04.json').read_bytes()).hexdigest()
    checks['fixed_route'] = {'passed': route['branches'] == 0 and 500 < route['length_m'] < 700,
                             'route_id': route['id'], 'length_m': route['length_m']}
    for condition in ('direct', 'hold', 'tirex'):
        path = ROOT / 'runs' / f'prep_{condition}_full_02'
        try:
            config = json.loads((path / 'config.json').read_text())
            summary = json.loads((path / 'summary.json').read_text())
            metrics = json.loads((path / 'metrics.json').read_text())
            bridge = json.loads((path / 'bridge-summary.json').read_text())
            passed = (summary['status'] == 'finished' and summary['completed'] and not summary['cleanup_errors']
                and not summary['collisions'] and summary['frame_mismatches'] == 0
                and summary['camera_frames'] > 100 and summary['displayed_images'] > 100
                and .97 <= summary['sim_wall_ratio'] <= 1.03 and summary['p95_tick_interval_ms'] < 15
                and config['diagnostic_input'] and config['route_sha256'] == route_hash
                and metrics['scored_ticks'] > 1000 and metrics['mean_grid_aoi_at_planned_use_ms'] < 400
                and bridge['status'] in ('finished', 'stopped'))
            if condition == 'tirex':
                passed = passed and metrics['control_margin_covered'] and metrics['inference_failures'] == 0
                passed = passed and metrics['trajectory_fraction'] > .95 and bridge['runtime_counts']['max_in_flight'] == 1
            if condition == 'hold':
                passed = passed and metrics['mean_grid_aoi_at_planned_use_ms'] >= 300
            checks[f'{condition}_full_route'] = {'passed': passed, 'directory': str(path),
                'scored_ticks': metrics['scored_ticks'], 'mean_tick_interval_ms': metrics['mean_tick_interval_ms'],
                'sim_wall_ratio': metrics['sim_wall_ratio'], 'trajectory_fraction': metrics['trajectory_fraction']}
        except (FileNotFoundError, KeyError) as exc:
            checks[f'{condition}_full_route'] = {'passed': False, 'error': str(exc)}
    external = json.loads((ROOT / 'logs/external-resources-audit.json').read_text())
    checks['external_resources_unchanged'] = {'passed': external['passed']}
    causal = json.loads((ROOT / 'logs/bridge-causality-audit.json').read_text())
    checks['recorded_causality_and_delay'] = {'passed': causal['passed']}
    interrupted = json.loads((ROOT / 'runs/prep_interrupt_cleanup/summary.json').read_text())
    checks['interrupt_cleanup'] = {'passed': interrupted['status'] == 'interrupted'
        and interrupted['exit_reason'] == 'user_stop' and not interrupted['cleanup_errors']}
    wheel = json.loads((ROOT / 'configs/wheel.json').read_text())
    result = {'status': 'ready_to_connect_wheel' if all(v['passed'] for v in checks.values()) else 'needs_work',
        'checked_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(), 'checks': checks,
        'wheel_mapping_measured': wheel['calibrated'],
        'pending_physical_steps': ['Connect T248P, PC mode, 360 degrees',
            'Measure pygame steering axis/sign with probe_wheel.sh --calibrate',
            'Confirm vehicle left/right and complete direct human practice']}
    (ROOT / 'logs/preflight.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['status'] == 'ready_to_connect_wheel' else 1


if __name__ == '__main__':
    raise SystemExit(main())
