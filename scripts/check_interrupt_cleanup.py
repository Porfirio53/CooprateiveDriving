"""Exercise graceful termination of a real, wheel-free CARLA trial."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', choices=('town03', 'town04', 'town05'))
    parser.add_argument('--condition', choices=('direct', 'hold', 'tirex', 'manual_tirex'), default='direct')
    parser.add_argument('--trial-id', default='prep_interrupt_cleanup')
    args = parser.parse_args()
    output = ROOT / 'runs' / args.trial_id
    if output.exists():
        raise RuntimeError('Interrupt diagnostic already exists; inspect it before re-running')
    with (ROOT / 'logs' / (output.name + '.log')).open('w') as log:
        command = ['bash', str(ROOT / 'scripts/run_trial.sh'), '--condition', args.condition,
            '--diagnostic-input', '--trial-id', output.name]
        if args.map:
            command += ['--map', args.map]
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 60
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Diagnostic exited before interrupt')
                ticks = output / 'ticks.csv'
                if ticks.exists() and ticks.stat().st_size > 5000:
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('Diagnostic startup timeout')
            process.send_signal(signal.SIGTERM)
            code = process.wait(timeout=30)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=30)
    summary = json.loads((output / 'summary.json').read_text())
    assert code == 0 and summary['status'] == 'interrupted' and summary['exit_reason'] == 'user_stop', summary
    assert not summary['cleanup_errors'], summary
    assert (output / 'trajectory.csv').exists() and (output / 'metrics.json').exists()
    import pandas as pd
    trajectory = pd.read_csv(output / 'trajectory.csv')
    assert len(trajectory) > 1 and (trajectory.frame.diff().dropna() == 1).all()
    import carla
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10)
    world = client.get_world()
    assert not world.get_settings().synchronous_mode
    assert not [a for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
    print('Interrupt cleanup passed: log saved, actors destroyed, asynchronous settings restored')


if __name__ == '__main__':
    main()
