"""Own the unchanged Frenet planner in a separate process, away from 100 Hz I/O."""
_planner = None


def initialize(config_data):
    global _planner
    import json
    from pathlib import Path
    import carla
    from omegaconf import OmegaConf
    from planner.vehicle_planner import VehiclePlan
    config = OmegaConf.create(config_data)
    route = json.loads(Path(config.simulator.route_file).read_text())
    start, end = route['start'], route['end']
    info = (None, [], carla.Location(x=start['x'], y=start['y'], z=start['z']),
            carla.Location(x=end['x'], y=end['y'], z=end['z']))
    _planner = VehiclePlan(config, info)


def ready():
    return _planner._global_path


def run_step(observation):
    return _planner.run_step(observation, '')
