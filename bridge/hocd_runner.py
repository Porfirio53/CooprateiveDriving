"""One paced CARLA owner, asynchronous planning/prediction and complete trial logs."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import datetime
import csv
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import numpy as np
from omegaconf import OmegaConf
import pygame
import torch

from .control_client import SteeringClient, monotonic_us
from . import planning_worker
from .experiment import MAPS, FORMAL_CONDITIONS, condition_settings, load_scene, terminal_state

ROOT = Path(__file__).resolve().parents[1]
HOCD = ROOT / 'vendor/HOCD'

FIELDS = ['trial_id', 'participant', 'condition', 'diagnostic_input', 'phase', 'tick',
    'wall_time_us', 'native_source_time_us', 'raw_steer', 'state_frame', 'sim_time_s',
    'planned_target_time_us', 'target_frame', 'command_sent_us', 'actual_apply_frame',
    'actual_tick_start_us', 'actual_tick_return_us', 'target_lateness_us',
    'last_observation', 'last_source_time_us', 'last_native_source_time_us', 'aoi_us',
    'human_steer_est', 'selection_mode', 'selection_reason', 'timing_valid', 'forecast_id',
    'forecast_source_time_us', 'selected_source_offset_us', 'compute_us',
    'x', 'y', 'z', 'yaw', 'speed_mps', 'automatic_steer', 'authority', 'mixed_steer',
    'applied_steer', 'throttle', 'brake', 'e_d_preview', 'e_fi_sin', 'lateral_error_m',
    'route_progress_m', 'collision_count', 'distance_to_end_m', 'control_work_us',
    'tick_interval_us', 'camera_frame']

TRAJECTORY_FIELDS = ['trial_id', 'participant', 'map', 'condition', 'diagnostic_input',
    'phase', 'tick', 'frame', 'sim_time_s', 'score_elapsed_s', 'wall_time_us',
    'tick_submit_us', 'tick_return_us', 'x_m', 'y_m', 'z_m', 'yaw_rad',
    'vx_mps', 'vy_mps', 'vz_mps', 'ax_mps2', 'ay_mps2', 'yaw_rate_radps',
    'lateral_error_m', 'route_progress_m',
    'previous_applied_steer', 'previous_throttle', 'previous_brake', 'collision_count']


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def signed_route_error(route, x, y):
    points = route[:, :2]
    ab = points[1:] - points[:-1]
    diff = np.array([x, y]) - points[:-1]
    weight = np.clip(np.sum(diff * ab, axis=1) / np.sum(ab * ab, axis=1), 0., 1.)
    projected = points[:-1] + weight[:, None] * ab
    residual = np.array([x, y]) - projected
    index = int(np.argmin(np.sum(residual * residual, axis=1)))
    length = np.linalg.norm(ab, axis=1)
    error = (ab[index, 0] * residual[index, 1] - ab[index, 1] * residual[index, 0]) / length[index]
    progress = float(length[:index].sum() + weight[index] * length[index])
    return float(error), progress


def diagnostic_steering(reference, ego):
    """Scripted lane follower used exclusively in marked deployment diagnostics."""
    points = reference[:, :2]
    nearest = int(np.argmin(np.sum((points - [ego.x, ego.y]) ** 2, axis=1)))
    lookahead = max(6., ego.vx * .9)
    distance = 0.
    index = nearest
    while index + 1 < len(points) and distance < lookahead:
        distance += float(np.linalg.norm(points[index + 1] - points[index]))
        index += 1
    dx, dy = points[index] - [ego.x, ego.y]
    alpha = math.atan2(dy, dx) - ego.yaw
    wheel_angle = math.atan2(2 * (ego.param.lf + ego.param.lr) * math.sin(alpha), max(distance, 1.))
    return float(np.clip(wheel_angle / .6, -1., 1.))


def run(args):
    from simulator.simulator import CarlaSimulator
    from controller.vehicle_controller import VehicleControl
    from display.pygame_display import PygameDisplay
    from utils.data import ControlInfo, LatDebugInfo, LonDebugInfo, PlanInfo

    os.chdir(HOCD)
    torch.set_num_threads(1)
    config = OmegaConf.load(HOCD / 'utils/config.yaml')
    settings = condition_settings(args.condition)
    route_path, scene = load_scene(args.map) if args.map else (ROOT / 'configs/route_town04.json', None)
    if scene:
        config.simulator.town = scene['town']
        config.simulator.route_file = str(route_path)
        config.controller.target_speed_kmh = scene['target_speed_kmh']
        config.planner.sync_observation = True
        config.planner.obstacle_lateral_limit_m = 9.
    if not settings['shared_control']:
        config.controller.lateral.type = 'DelayedManualControl'
    wheel = json.loads((ROOT / 'configs/wheel.json').read_text())
    if not args.diagnostic_input and not wheel.get('calibrated'):
        raise RuntimeError('T248P mapping not measured. Run bash scripts/probe_wheel.sh --calibrate after connecting.')
    for key in ('device_index', 'steering_axis', 'steering_sign'):
        config.controller.lateral.G29[key] = wheel[key]
    config.controller.lateral.G29.expected_name = wheel.get('device_name')
    config.controller.lateral.G29.expected_guid = wheel.get('guid')
    config.controller.lateral.G29.diagnostic_input = args.diagnostic_input
    trial = args.trial_id or datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f') + '_' + args.condition
    if any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in trial):
        raise ValueError('Trial ID must be alphanumeric, underscore or hyphen')
    output = ROOT / 'runs' / trial
    output.mkdir(parents=True, exist_ok=False)
    route_config = json.loads(route_path.read_text())
    reference = np.array(route_config['waypoints'], dtype=float)
    metadata = {'trial_id': trial, 'participant': args.participant, 'repeat': args.repeat,
        'condition': args.condition, 'diagnostic_input': args.diagnostic_input,
        **settings, 'map': args.map or 'legacy_town04',
        'experiment_id': 'single_driver_3maps_v1' if args.formal else None,
        'formal': args.formal, 'scenario_id': route_config['id'],
        'created_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'wheel': wheel, 'route': route_config, 'route_sha256': file_hash(route_path),
        'config': OmegaConf.to_container(config, resolve=True),
        'hocd_revision': subprocess.check_output(['git', '-C', str(HOCD), 'rev-parse', 'HEAD'], text=True).strip(),
        'hocd_patch_sha256': hashlib.sha256(subprocess.check_output(['git', '-C', str(HOCD), 'diff', 'HEAD'])).hexdigest(),
        'hocd_source_sha256': {str(p.relative_to(HOCD)): file_hash(p)
            for directory in ('controller', 'display', 'planner', 'simulator', 'rl', 'utils')
            for p in sorted((HOCD / directory).rglob('*.py'))},
        'bridge_code_sha256': {str(p.relative_to(ROOT)): file_hash(p) for p in sorted((ROOT / 'bridge').glob('*.py'))},
        'tirex_model': '/home/patton/MyTirex/models/tirex2',
        'tirex_source': '/home/patton/MyTirex/vendor/tirex', 'sampling_period_us': 25_000,
        'control_period_us': 10_000, 'pre_ts_s': .2, 'warmup_min_s': 7.,
        'max_seconds': args.max_seconds, 'status': 'initializing',
        'application_time_definition': 'Next world.tick submission and return bracket physical application; actual frame is observed, not assumed.'}
    (output / 'config.json').write_text(json.dumps(metadata, indent=2) + '\n')
    (output / 'scenario.json').write_text(json.dumps(route_config, indent=2) + '\n')
    np.savetxt(output / 'scoring_reference.csv', reference, delimiter=',', header='x_m,y_m,z_m', comments='')
    print(f'Run output: {output}', flush=True)
    client = sim = controller = display = bridge_process = None
    planner_executor = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
        initializer=planning_worker.initialize, initargs=(OmegaConf.to_container(config, resolve=True),))
    bridge_log = None
    summary = {'trial_id': trial, 'status': 'initializing', 'completed': False, 'exit_reason': None}
    pending = None
    control_rows = []
    plan_future = None
    start = None
    score_start = None
    last_score_elapsed = 0.

    def stop_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_signal)
    try:
        if scene:
            subprocess.run([sys.executable, '-B', str(ROOT / 'scripts/carla_service.py'),
                            'start', '--town', scene['town']], check=True, cwd=ROOT)
        # Open the output socket before starting the bridge, so no observations
        # or trajectories are lost while the model is being loaded.
        client = SteeringClient(trial, settings['bridge_condition'])
        bridge_log = (output / 'bridge-console.log').open('w')
        bridge_process = subprocess.Popen([os.environ['TIREX_PYTHON'], '-B', '-m', 'bridge.tirex_bridge',
            '--trial-id', trial, '--condition', settings['bridge_condition'], '--run-dir', str(output),
            '--duration-s', str(args.max_seconds + 180)], cwd=ROOT, stdout=bridge_log, stderr=subprocess.STDOUT)
        timeout = time.monotonic() + 180
        while not (output / 'bridge-ready.json').exists():
            if bridge_process.poll() is not None:
                raise RuntimeError('Bridge failed to start; see bridge-console.log')
            if time.monotonic() >= timeout:
                raise RuntimeError('Bridge startup timeout')
            time.sleep(.05)
        display = PygameDisplay(config)
        controller = VehicleControl(config)
        # Retain the instance even if setup fails, so partial actors are cleaned.
        sim = CarlaSimulator.__new__(CarlaSimulator)
        sim.__init__(config, init_level='full')
        (output / 'scene.json').write_text(json.dumps(sim.scene_snapshot(), indent=2) + '\n')
        if scene:
            display.set_scene(route_config)
        global_path = planner_executor.submit(planning_worker.ready).result(timeout=30)
        np.savetxt(output / 'reference_route.csv', np.array(global_path),
                   delimiter=',', header='x,y,yaw,curvature', comments='')
        metadata.update(status='running', carla_server_version=sim.client.get_server_version())
        (output / 'config.json').write_text(json.dumps(metadata, indent=2) + '\n')
        ctrl = ControlInfo(lat=LatDebugInfo(), lon=LonDebugInfo(), steer=0., throttle=0., brake=.5)
        plan = None
        start = monotonic_us()
        next_tick = start
        score_start = None
        tick = 0
        next_plan = 0
        previous_tick_start = None
        with (output / 'ticks.csv').open('w', newline='', buffering=1) as stream, \
                (output / 'trajectory.csv').open('w', newline='', buffering=1) as trajectory_stream:
            writer = csv.DictWriter(stream, FIELDS)
            writer.writeheader()
            trajectory_writer = csv.DictWriter(trajectory_stream, TRAJECTORY_FIELDS)
            trajectory_writer.writeheader()
            while True:
                remaining = next_tick - monotonic_us()
                if remaining > 0:
                    time.sleep(remaining / 1e6)
                env = sim.tick()
                tick_start, tick_return = sim.last_tick_start_us, sim.last_tick_return_us
                now = monotonic_us()
                if pending is not None:
                    pending.update(actual_apply_frame=sim.last_frame,
                        actual_tick_start_us=tick_start, actual_tick_return_us=tick_return,
                        target_lateness_us=tick_start - pending['planned_target_time_us'])
                    # Ground truth at physical tick submission is sample-and-hold:
                    # the preceding raw sample, captured before that submission.
                    pending['collision_count'] = len(sim.collision_hist)
                    writer.writerow(pending)
                    if pending['phase'] == 'score':
                        control_rows.append(pending)
                    pending = None
                # Persist each observed physics frame before any quit event,
                # input acquisition, prediction or planner call can interrupt.
                error, progress = signed_route_error(reference, env.ego.x, env.ego.y)
                last_score_elapsed = (now - score_start) / 1e6 if score_start is not None else 0.
                trajectory_writer.writerow({'trial_id': trial, 'participant': args.participant,
                    'map': metadata['map'], 'condition': args.condition, 'diagnostic_input': args.diagnostic_input,
                    'phase': 'score' if score_start is not None else 'warmup', 'tick': tick,
                    'frame': sim.last_frame, 'sim_time_s': sim.snapshot.timestamp.elapsed_seconds,
                    'score_elapsed_s': last_score_elapsed if score_start is not None else None,
                    'wall_time_us': now, 'tick_submit_us': tick_start, 'tick_return_us': tick_return,
                    'x_m': env.ego.x, 'y_m': env.ego.y, 'z_m': env.ego.z, 'yaw_rad': env.ego.yaw,
                    'vx_mps': env.ego.vx, 'vy_mps': env.ego.vy, 'vz_mps': env.ego.vz,
                    'ax_mps2': env.ego.ax, 'ay_mps2': env.ego.ay, 'yaw_rate_radps': env.ego.yaw_rate,
                    'lateral_error_m': error, 'route_progress_m': progress,
                    'previous_applied_steer': ctrl.steer, 'previous_throttle': ctrl.throttle,
                    'previous_brake': ctrl.brake, 'collision_count': len(sim.collision_hist)})
                display.pump()
                if bridge_process.poll() is not None:
                    raise RuntimeError('bridge_stopped')
                client.drain()
                # This deadline is fixed before control/planning; late ticks
                # retain their original planned time and target frame in CSV.
                t_use = next_tick + 10_000
                if t_use <= now:
                    t_use = now + 10_000
                native_time = monotonic_us()
                raw = diagnostic_steering(reference, env.ego) if args.diagnostic_input else controller.lat_control.driver_control.sample()
                client.send_raw(raw, native_time)
                selection = client.select(t_use)
                if score_start is None:
                    ready = now - start >= 7_000_000 and client.context_ready
                    if settings['prediction']:
                        ready = ready and selection.mode == 'trajectory'
                    if ready:
                        if sim.camera_frames == 0:
                            raise RuntimeError('camera_has_no_frames')
                        score_start = now
                        print('History ready; driving / scoring starts.', flush=True)
                    elif now - start > 25_000_000:
                        raise RuntimeError('history_or_first_forecast_timeout')
                # Check terminal physics states before a planner error can
                # obscure a collision/off-route/end result. Trajectory is saved.
                terminal = terminal_state(collisions=sim.collision_hist,
                    lateral_error=error, progress=progress, route_length=route_config['length_m'],
                    distance_to_end=math.hypot(env.ego.x - reference[-1, 0], env.ego.y - reference[-1, 1]),
                    elapsed=(now - score_start) / 1e6 if score_start is not None else None,
                    max_seconds=args.max_seconds, diagnostic=args.diagnostic_input,
                    off_route_limit=route_config.get('off_route_limit_m', 5.),
                    finish_distance=route_config.get('finish_distance_m', 7.))
                if terminal:
                    summary.update(terminal)
                    break
                if score_start is not None:
                    if not settings['shared_control']:
                        # Reference is only for the HUD, metrics and PID call;
                        # no automatic lateral planner constrains human control.
                        plan = PlanInfo(global_path=global_path, local_path=global_path,
                            speed=[route_config['target_speed_kmh'] / 3.6] * len(global_path),
                            distance=math.hypot(env.ego.x - reference[-1, 0], env.ego.y - reference[-1, 1]))
                    else:
                        if plan_future is not None and plan_future.done():
                            plan = plan_future.result()
                            plan_future = None
                        if now >= next_plan and plan_future is None:
                            plan_future = planner_executor.submit(planning_worker.run_step, replace(env, image=None))
                            next_plan = now + 200_000
                    if plan is not None:
                        if selection.mode == 'unavailable':
                            raise RuntimeError('no_delayed_observation')
                        ctrl = controller.run_step(env, plan, human_steer=selection.value)
                work_start = now
                frame = sim.last_frame
                mixed = (ctrl.lat.authority * ctrl.lat.steer_vehicle + (1 - ctrl.lat.authority) * ctrl.lat.steer_driver
                         if ctrl.lat.authority is not None else 0.)
                row = {'trial_id': trial, 'participant': args.participant, 'condition': args.condition,
                    'diagnostic_input': args.diagnostic_input, 'phase': 'score' if score_start is not None else 'warmup',
                    'tick': tick, 'wall_time_us': now, 'native_source_time_us': native_time, 'raw_steer': raw,
                    'state_frame': frame, 'sim_time_s': sim.snapshot.timestamp.elapsed_seconds,
                    'planned_target_time_us': t_use, 'target_frame': frame + 1,
                    'last_observation': selection.hold_value, 'last_source_time_us': selection.source_time_us,
                    'last_native_source_time_us': selection.native_source_time_us, 'aoi_us': selection.aoi_us,
                    'human_steer_est': selection.value, 'selection_mode': selection.mode,
                    'selection_reason': selection.reason, 'timing_valid': selection.timing_valid,
                    'forecast_id': selection.forecast_id, 'forecast_source_time_us': selection.forecast_source_time_us,
                    'selected_source_offset_us': selection.source_offset_us, 'compute_us': selection.compute_us,
                    'x': env.ego.x, 'y': env.ego.y, 'z': env.ego.z, 'yaw': env.ego.yaw, 'speed_mps': env.ego.vx,
                    'automatic_steer': ctrl.lat.steer_vehicle, 'authority': ctrl.lat.authority,
                    'mixed_steer': mixed, 'applied_steer': ctrl.steer, 'throttle': ctrl.throttle, 'brake': ctrl.brake,
                    'e_d_preview': ctrl.lat.e_d, 'e_fi_sin': ctrl.lat.e_fi,
                    'lateral_error_m': error, 'route_progress_m': progress,
                    'collision_count': len(sim.collision_hist), 'distance_to_end_m': plan.distance if plan else None,
                    'tick_interval_us': tick_start - previous_tick_start if previous_tick_start else None,
                    'camera_frame': sim.last_camera_frame}
                sim.control_vehicle(throttle=ctrl.throttle, steer=ctrl.steer, brake=ctrl.brake)
                row['command_sent_us'] = monotonic_us()
                row['control_work_us'] = monotonic_us() - work_start
                pending = row
                status = f"{metadata['map'].upper()} | {args.condition.upper()} | "
                status += (f"{(now - score_start) / 1e6:.1f}s | {progress:.0f}/{route_config['length_m']:.0f}m"
                           if score_start is not None else 'Preparing history...')
                display.run_step(env, ctrl, status)
                previous_tick_start = tick_start
                next_tick = t_use
                tick += 1
        summary['status'] = 'finished'
    except KeyboardInterrupt:
        summary.update(status='interrupted', exit_reason='user_stop')
    except BaseException as exc:
        summary.update(status='failed', exit_reason=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        cleanup_errors = []
        if sim is not None and getattr(sim, 'ego_vehicle', None) is not None:
            try:
                sim.control_vehicle(brake=1.)
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if plan_future is not None:
            plan_future.cancel()
        planner_executor.shutdown(wait=True, cancel_futures=True)
        if sim is not None:
            try:
                sim.destroy_actors()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if controller is not None:
            try:
                controller.lat_control.driver_control.close()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if args.save_preview and sim is not None and getattr(sim, 'fps_image', None) is not None:
            try:
                pygame.image.save(pygame.surfarray.make_surface(sim.fps_image.swapaxes(0, 1)), str(output / 'camera-preview.png'))
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if display is not None:
            display.close()
        pygame.quit()
        if client is not None:
            summary['rejected_replies'] = client.rejected
            client.close()
        if bridge_process is not None and bridge_process.poll() is None:
            bridge_process.terminate()
            try:
                bridge_process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                bridge_process.kill()
                bridge_process.wait()
        if bridge_log is not None:
            bridge_log.close()
        if sim is not None:
            summary.update(camera_frames=getattr(sim, 'camera_frames', 0),
                displayed_images=display.image_count if display else 0,
                collisions=getattr(sim, 'collision_hist', []))
        summary['cleanup_errors'] = cleanup_errors
        summary['scored_ticks'] = len(control_rows)
        summary['wall_duration_s'] = (monotonic_us() - start) / 1e6 if start else 0
        summary['scoring_duration_s'] = last_score_elapsed
        if control_rows:
            intervals = [r['tick_interval_us'] for r in control_rows if r['tick_interval_us']]
            summary.update(lateral_rmse_m=float(np.sqrt(np.mean([r['lateral_error_m'] ** 2 for r in control_rows]))),
                mean_tick_interval_ms=float(np.mean(intervals) / 1000),
                p95_tick_interval_ms=float(np.percentile(intervals, 95) / 1000),
                sim_wall_ratio=(control_rows[-1]['sim_time_s'] - control_rows[0]['sim_time_s']) /
                    max(1e-6, (control_rows[-1]['wall_time_us'] - control_rows[0]['wall_time_us']) / 1e6),
                trajectory_fraction=sum(r['selection_mode'] == 'trajectory' for r in control_rows) / len(control_rows),
                mean_observation_aoi_ms=float(np.mean([r['aoi_us'] for r in control_rows if r['aoi_us'] is not None]) / 1000),
                frame_mismatches=sum(r['target_frame'] != r.get('actual_apply_frame') for r in control_rows))
        metadata['status'] = summary['status']
        (output / 'config.json').write_text(json.dumps(metadata, indent=2) + '\n')
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        try:
            from scripts.analyze_run import analyze
            analyze(output)
        except Exception as exc:
            summary['artifact_error'] = str(exc)
            (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--condition', choices=('direct', *FORMAL_CONDITIONS), default='direct')
    parser.add_argument('--participant')
    parser.add_argument('--map', choices=MAPS)
    parser.add_argument('--formal', action='store_true')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--max-seconds', type=float, default=90.)
    parser.add_argument('--trial-id')
    parser.add_argument('--diagnostic-input', action='store_true')
    parser.add_argument('--save-preview', action='store_true', help='Save one final camera image, never an image sequence')
    args = parser.parse_args()
    if args.formal and (not args.map or args.condition not in FORMAL_CONDITIONS or args.diagnostic_input):
        parser.error('Formal run requires --map town03/town04/town05 and --condition hold/tirex/manual_tirex, with live wheel input')
    if args.formal and args.max_seconds != 90:
        parser.error('Formal runs keep a 90 second limit; expected route duration is 30–45 seconds')
    if args.condition == 'manual_tirex' and not args.map:
        parser.error('manual_tirex requires one of the three --map courses')
    args.participant = args.participant or ('P001' if args.formal else 'PILOT')
    if not 1 <= args.max_seconds <= 180:
        parser.error('max-seconds must be in 1..180')
    import fcntl
    (ROOT / 'runtime').mkdir(exist_ok=True)
    with (ROOT / 'runtime/trial.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('A trial is already running; finish it before starting another')
        run(args)
