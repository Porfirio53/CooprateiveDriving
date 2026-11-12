"""Freeze three drivable obstacle courses from the installed CARLA 0.9.16 maps.

Preparation only: no ego car or human trial is started. Regeneration is explicit.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

import carla
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def trace(seed):
    points = [seed]
    for _ in range(150):
        options = points[-1].next(2.)
        if not options:
            return None
        previous = points[-1].transform.rotation.yaw
        points.append(min(options, key=lambda q: abs((q.transform.rotation.yaw - previous + 180) % 360 - 180)))
    return points


def bypass_options(points, distances, center):
    options = []
    for side in (-1, 1):
        widths = []
        for point, distance in zip(points, distances):
            if abs(distance - center) > 37:
                continue
            neighbor = point.get_left_lane() if side < 0 else point.get_right_lane()
            if (neighbor is None or neighbor.lane_type != carla.LaneType.Driving
                    or neighbor.is_junction or point.is_junction
                    or abs((neighbor.transform.rotation.yaw - point.transform.rotation.yaw + 180) % 360 - 180) > 15):
                break
            widths.append((neighbor.lane_width + point.lane_width) / 2)
        else:
            if widths and min(widths) >= 3.2:
                options.append((side, min(widths)))
    return options


def smooth_bump(distance, center):
    gap = abs(distance - center)
    if gap >= 36:
        return 0.
    if gap <= 10:
        return 1.
    t = (36 - gap) / 26
    return t ** 3 * (10 - 15 * t + 6 * t * t)


def create_scene(town_map, town, force=False):
    output = ROOT / 'configs/scenarios' / (town.lower() + '.json')
    if output.exists() and not force:
        raise FileExistsError(f'{output}: use --force only to deliberately change the frozen course')
    assert town_map.name.split('/')[-1].removesuffix('_Opt') == town
    candidates = []
    centers = (85., 195.)
    for index, seed in enumerate(town_map.generate_waypoints(25.)):
        points = trace(seed)
        if points is None:
            continue
        distances = np.r_[0., np.cumsum([a.transform.location.distance(b.transform.location)
                                      for a, b in zip(points, points[1:])])]
        changes = [abs((b.transform.rotation.yaw - a.transform.rotation.yaw + 180) % 360 - 180)
                   for a, b in zip(points, points[1:])]
        if max(changes) > 5 or points[0].transform.location.distance(points[-1].transform.location) < 160:
            continue
        options = [bypass_options(points, distances, center) for center in centers]
        if not all(options):
            continue
        selected = [options[0][0], next((o for o in options[1] if o[0] != options[0][0][0]), options[1][0])]
        obstacles = []
        for number, (center, (side, width)) in enumerate(zip(centers, selected), 1):
            point = points[int(np.argmin(abs(distances - center)))]
            t = point.transform
            right = t.get_right_vector()
            obstacles.append({'id': f'parked_{number}', 'model': 'vehicle.audi.tt',
                'color': '210,60,30' if number == 1 else '240,185,30',
                'x': t.location.x - side * .4 * right.x,
                'y': t.location.y - side * .4 * right.y, 'z': t.location.z + .15,
                'yaw': t.rotation.yaw, 'route_station_m': float(distances[int(np.argmin(abs(distances - center)))]),
                'bypass_side': 'left' if side < 0 else 'right', 'bypass_offset_m': side * width,
                'static': True})
        reference = []
        drivable = True
        for point, distance in zip(points, distances):
            t = point.transform
            offset = sum(side * width * smooth_bump(distance, center)
                         for center, (side, width) in zip(centers, selected))
            right = t.get_right_vector()
            location = carla.Location(x=t.location.x + offset * right.x,
                y=t.location.y + offset * right.y, z=t.location.z)
            lane = town_map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Driving)
            if lane is None:
                drivable = False
                break
            reference.append([location.x, location.y, location.z])
        if not drivable:
            continue
        xy = np.array(reference)[:, :2]
        gap = min(float(np.linalg.norm(xy - [o['x'], o['y']], axis=1).min()) for o in obstacles)
        if gap < 3.35:
            continue
        junctions = sum(p.is_junction for p in points)
        score = abs(sum(changes) - {'Town03': 60, 'Town04': 45, 'Town05': 70}[town]) + junctions * 2
        if selected[0][0] == selected[1][0]:
            score += 15
        candidates.append((score, index, points, reference, obstacles, gap, sum(changes)))
    if not candidates:
        raise RuntimeError(f'No drivable obstacle course found in {town}')
    score, index, points, reference, obstacles, gap, turn = min(candidates, key=lambda c: c[0])

    def position(point):
        t = point.transform
        return dict(x=t.location.x, y=t.location.y, z=t.location.z, yaw=t.rotation.yaw)

    length = float(np.linalg.norm(np.diff(np.array(reference)[:, :2], axis=0), axis=1).sum())
    asset = 'Town03_Opt' if town == 'Town03' else town
    scene = {'id': town + '_obstacles_v1', 'town': asset, 'carla_version': '0.9.16',
        'discovery_spawn_index': index, 'start': position(points[0]), 'end': position(points[-1]),
        'length_m': length, 'target_speed_kmh': 30., 'expected_scoring_seconds': [30, 45],
        'max_seconds': 90., 'off_route_limit_m': 5., 'finish_distance_m': 5.,
        'weather': 'ClearNoon', 'obstacles': obstacles, 'waypoints': reference,
        'base_lane_waypoints': [[p.transform.location.x, p.transform.location.y, p.transform.location.z] for p in points],
        'min_reference_to_obstacle_center_m': gap, 'base_heading_change_degrees': turn,
        'road_lane_sequence': list(dict.fromkeys(f'{p.road_id}:{p.lane_id}' for p in points)),
        'note': 'Fixed reference uses an adjacent lane to bypass two parked cars. All three conditions share this scene. No moving traffic.'}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scene, indent=2) + '\n')
    print(json.dumps({k: scene[k] for k in ('id', 'discovery_spawn_index', 'length_m', 'min_reference_to_obstacle_center_m', 'base_heading_change_degrees')}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--town', choices=('Town03', 'Town04', 'Town05'), action='append')
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--offline', action='store_true', help='Read installed OpenDRIVE geometry without starting CARLA')
    args = parser.parse_args()
    for town in args.town or ('Town03', 'Town04', 'Town05'):
        if args.offline:
            path = Path('/home/patton/carla_experiments/simulator/CarlaUE4/Content/Carla/Maps/OpenDrive') / (town + '.xodr')
            town_map = carla.Map(town, path.read_text())
        else:
            asset = 'Town03_Opt' if town == 'Town03' else town
            subprocess.run([sys.executable, '-B', str(ROOT / 'scripts/carla_service.py'), 'start', '--town', asset], check=True)
            client = carla.Client('127.0.0.1', 2000)
            client.set_timeout(20.)
            assert not [a for a in client.get_world().get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
            town_map = client.get_world().get_map()
        create_scene(town_map, town, args.force)


if __name__ == '__main__':
    main()
