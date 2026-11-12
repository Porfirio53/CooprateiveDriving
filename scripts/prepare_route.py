"""Freeze an unambiguous gentle Town04 lane route in CARLA 0.9.16."""
import json
import math
from pathlib import Path
import carla

ROOT = Path(__file__).resolve().parents[1]


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(60)
    world = client.get_world()
    if world.get_map().name.split('/')[-1] != 'Town04':
        world = client.load_world('Town04')
    town = world.get_map()
    seed_index = 150
    waypoint = town.get_waypoint(town.get_spawn_points()[seed_index].location)
    points, headings, branches = [waypoint], [], 0
    for _ in range(300):
        options = waypoint.next(2.)
        if not options:
            raise RuntimeError('Route ended before 600 m')
        branches += len(options) > 1
        next_waypoint = min(options, key=lambda p: abs((p.transform.rotation.yaw - waypoint.transform.rotation.yaw + 180) % 360 - 180))
        headings.append(abs((next_waypoint.transform.rotation.yaw - waypoint.transform.rotation.yaw + 180) % 360 - 180))
        waypoint = next_waypoint
        points.append(waypoint)
    if branches or max(headings) > 2.:
        raise RuntimeError('Expected one unambiguous lane with gentle bends')

    def position(point):
        transform = point.transform
        return {'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z,
                'yaw': transform.rotation.yaw, 'road_id': point.road_id, 'lane_id': point.lane_id, 's': point.s}

    length = sum(a.transform.location.distance(b.transform.location) for a, b in zip(points, points[1:]))
    route = {'id': 'Town04_lane_seed150_600m_v1', 'carla_version': client.get_server_version(),
        'town': 'Town04', 'discovery_spawn_index': seed_index,
        'start': position(points[0]), 'end': position(points[-1]),
        'length_m': length, 'max_yaw_step_degrees_per_2m': max(headings),
        'branches': branches, 'target_speed_kmh': 30.,
        'waypoints': [[p.transform.location.x, p.transform.location.y, p.transform.location.z] for p in points[::2]],
        'road_lane_sequence': list(dict.fromkeys(f'{p.road_id}:{p.lane_id}' for p in points)),
        'note': 'Recorded coordinates are authoritative; the discovery index is provenance only. No dynamic obstacles.'}
    target = ROOT / 'configs/route_town04.json'
    target.write_text(json.dumps(route, indent=2) + '\n')
    print(json.dumps({k: v for k, v in route.items() if k != 'waypoints'}, indent=2))


if __name__ == '__main__':
    main()
