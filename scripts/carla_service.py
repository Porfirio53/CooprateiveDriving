"""Manage only the CARLA instance writing to this workspace's runtime directory."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def owned_processes():
    found = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            args = (path / 'cmdline').read_bytes().split(b'\0')
            if (args and args[0].endswith(b'/CarlaUE4-Linux-Shipping')
                    and f'-userdir={ROOT}/runtime/carla'.encode() in args):
                found.append(int(path.name))
        except (OSError, PermissionError):
            pass
    return found


def healthy():
    try:
        import carla
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.)
        return client.get_server_version() == '0.9.16'
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('start', 'status', 'stop'), default='status', nargs='?')
    parser.add_argument('--town', choices=('Town03', 'Town03_Opt', 'Town04', 'Town05'))
    args = parser.parse_args()
    pids = owned_processes()
    if args.action == 'status':
        state = {'owned_pids': pids, 'carla_0_9_16_healthy': healthy()}
        print(json.dumps(state))
        return 0 if pids and state['carla_0_9_16_healthy'] else 1
    if args.action == 'start':
        if pids and healthy():
            import carla
            client = carla.Client('127.0.0.1', 2000)
            client.set_timeout(5.)
            world = client.get_world()
            if not args.town or world.get_map().name.split('/')[-1] == args.town:
                print('CARLA 0.9.16 is ready, pid=' + ','.join(map(str, pids)))
                return 0
            if any(a.type_id.startswith(('vehicle.', 'sensor.')) for a in world.get_actors()):
                raise RuntimeError('End the current trial before changing maps')
            # A fresh server avoids UE4 crashes seen on in-process map reloads.
            for pid in pids:
                os.kill(pid, signal.SIGTERM)
            deadline = time.monotonic() + 20
            while owned_processes() and time.monotonic() < deadline:
                time.sleep(.2)
            pids = owned_processes()
        if pids:
            raise RuntimeError('Workspace CARLA is running but not responding; inspect logs/carla.log')
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1', 2000)) == 0:
                raise RuntimeError('Port 2000 is occupied by another server')
        log = (ROOT / 'logs/carla-console.log').open('a')
        environment = dict(os.environ)
        if args.town:
            environment['COOP_CARLA_MAP'] = '/Game/Carla/Maps/' + args.town
            settings_path = ROOT / 'runtime/carla/Saved/Config/LinuxNoEditor/Engine.ini'
            import re
            contents = settings_path.read_text() if settings_path.exists() else ''
            # UE ini files contain repeated array keys; preserve their text.
            section = '[/Script/EngineSettings.GameMapsSettings]'
            pattern = r'(?ms)^\[/Script/EngineSettings.GameMapsSettings\]\n.*?(?=^\[|\Z)'
            replacement = section + '\n' + ''.join(
                f'{key}={environment["COOP_CARLA_MAP"]}.{args.town}\n'
                for key in ('GameDefaultMap', 'ServerDefaultMap', 'TransitionMap')) + '\n'
            contents = re.sub(pattern, '', contents)
            system_pattern = r'(?ms)^\[SystemSettings\]\n.*?(?=^\[|\Z)'
            contents = re.sub(system_pattern, '', contents)
            contents += '\n' + replacement
            settings_path.parent.mkdir(parents=True, exist_ok=True)
            settings_path.write_text(contents)
        process = subprocess.Popen(['bash', str(ROOT / 'scripts/start_carla.sh')], cwd=ROOT, env=environment,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (ROOT / 'runtime/carla.pid').write_text(str(process.pid) + '\n')
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('CARLA exited; inspect logs/carla-console.log')
            if healthy():
                import carla
                client = carla.Client('127.0.0.1', 2000)
                client.set_timeout(5.)
                town = client.get_world().get_map().name.split('/')[-1]
                if not args.town or town == args.town:
                    print(f'CARLA 0.9.16 is ready, town={town}, pid={process.pid}')
                    return 0
            time.sleep(.5)
        raise RuntimeError('CARLA startup timeout; inspect logs/carla.log')
    # Refuse to stop a simulator while an experiment owns live actors.
    if healthy():
        import carla
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(3.)
        actors = client.get_world().get_actors()
        if any(a.type_id.startswith(('vehicle.', 'sensor.')) for a in actors):
            raise RuntimeError('CARLA still has vehicle/sensor actors; end the trial first')
    for pid in pids:
        __import__('os').kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 20
    while owned_processes() and time.monotonic() < deadline:
        time.sleep(.2)
    if owned_processes():
        raise RuntimeError('CARLA did not stop; inspect workspace logs')
    print('Stopped workspace CARLA' if pids else 'Workspace CARLA is already stopped')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
