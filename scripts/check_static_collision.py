"""Verify frozen obstacle vehicles still generate actual CARLA collisions."""
import json
from pathlib import Path
import sys
import time

import carla
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'vendor/HOCD')]
from simulator.simulator import CarlaSimulator


def main():
    config = OmegaConf.load(ROOT / 'configs/hocd.yaml')
    config.simulator.town = 'Town03_Opt'
    config.simulator.route_file = str(ROOT / 'configs/scenarios/town03.json')
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.)
    world = client.get_world()
    assert world.get_map().name.endswith('Town03_Opt')
    assert not [a for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
    simulator = CarlaSimulator.__new__(CarlaSimulator)
    try:
        simulator.__init__(config, init_level='full')
        # Move one diagnostic obstacle onto the straight departure so the
        # physics collision sensor, rather than path planning, is exercised.
        transform = simulator.ego_vehicle.get_transform()
        forward = transform.get_forward_vector()
        target = carla.Transform(carla.Location(x=transform.location.x + 9 * forward.x,
            y=transform.location.y + 9 * forward.y, z=transform.location.z), transform.rotation)
        simulator.other_vehicles[0].set_transform(target)
        for _ in range(600):
            started = time.monotonic()
            simulator.control_vehicle(throttle=.75, steer=0., brake=0.)
            simulator.tick()
            if simulator.collision_hist:
                break
            time.sleep(max(0., .01 - (time.monotonic() - started)))
        collisions = list(simulator.collision_hist)
        assert any(e['other_actor'] == 'vehicle.audi.tt' for e in collisions), collisions
        report = {'passed': True, 'diagnostic_only': True, 'collisions': collisions,
                  'note': 'A program-driven vehicle hit a physics-disabled parked obstacle; no wheel was read.'}
    finally:
        simulator.destroy_actors()
    assert not world.get_settings().synchronous_mode
    assert not [a for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'sensor.'))]
    (ROOT / 'logs/static-obstacle-collision-audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
