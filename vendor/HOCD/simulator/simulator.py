import carla
import numpy as np
import json
from pathlib import Path
import time

from utils.data import SimulatorObservation
from simulator.simulator_utils import get_vehicle_state


class CarlaSimulator:
    def __init__(self, config, init_level="basic"):
        self._load_config(config)
        self._setup()
        if init_level == "full":
            self._setup_other()

    def _load_config(self, config):
        cfg = config.simulator
        self._host = cfg.host
        self._port = cfg.port
        self._timeout = cfg.timeout
        self._town = cfg.town
        self._user_defined_spawn_points = cfg.user_defined_spawn_points
        self._synchronous = cfg.synchronous
        self._fixed_time= cfg.fixed_time
        self._no_rendering = cfg.no_rendering
        self.start_point = cfg.start_point
        self.end_point = cfg.end_point
        self._sampling_resolution = cfg.sampling_resolution
        self._vehicles = config.vehicles
        self._route = json.loads(Path(cfg.route_file).read_text()) if cfg.get('route_file') else None
        self._actors = []
        self.fps_camera = self.collision_sensor = self.ego_vehicle = None
        self.camera_frames = 0
        self.last_camera_frame = None
        self._camera_size = (int(cfg.get('camera_width', 640)), int(cfg.get('camera_height', 360)))

    def _setup(self):
        # Connect to CARLA server and load world
        self.client = carla.Client(self._host, self._port)
        self.client.set_timeout(self._timeout)
        self.world = self.client.get_world()
        if self.world.get_map().name.split('/')[-1] != self._town:
            self.world = self.client.load_world(self._town)
        self._original_settings = self.world.get_settings()
        self._original_weather = self.world.get_weather()
        if self._route and self._route.get('weather'):
            self.world.set_weather(getattr(carla.WeatherParameters, self._route['weather']))

        # Get blueprint library and map
        self.bp_lib = self.world.get_blueprint_library()
        self.town_map = self.world.get_map()

        # Build topology for global planning
        self.topology = [] if self._route else self._build_topology()
        
        # Generate spawn points
        if self._user_defined_spawn_points:
            # Generate spawn points with fixed spacing
            self.all_default_spawn = list(map(lambda x: x.transform, self.town_map.generate_waypoints(20.0)))
        else:
            # Default spawn points
            self.all_default_spawn = list(self.town_map.get_spawn_points())
        
        # Set synchronous mode and fixed simulation step
        self.set_synchronous_mode()

        self.other_vehicles = []
        self.collision_hist = []

    def _setup_other(self):
        
        # Set traffic manager
        if self._vehicles.get('obstacles', []):
            self._set_traffic_manager()

        # Spawn ego and other vehicles
        self._spawn_ego_vehicle()
        self._spawn_other_vehicle()

        # Add camera sensor
        self.fps_image = None
        self.fps_camera = self._spawn_camera((0.6, 0, 1.3))
        self.generate_collision_sensor()

        # Update world once
        self.tick()
        
    def _spawn_camera(self, location):
        """
        Create RGB camera and attach to ego vehicle
        """
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self._camera_size[0]))
        camera_bp.set_attribute('image_size_y', str(self._camera_size[1]))
        camera_bp.set_attribute('sensor_tick', '0.033333333')
        camera_bp.set_attribute('enable_postprocess_effects', 'false')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=location[0], y=location[1], z=location[2]))
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)
        self._actors.append(camera)
        camera.listen(lambda image: self.process_image(image))
        return camera

    def process_image(self, image):
        """
        Create RGB camera and attach to ego vehicle
        """
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        self.fps_image = array[:, :, :3][:, :, ::-1].copy()
        self.camera_frames += 1
        self.last_camera_frame = image.frame

    def set_synchronous_mode(self):
        """
        Enable synchronous simulation mode
        """
        setting = self.world.get_settings()
        setting.synchronous_mode = self._synchronous
        setting.no_rendering_mode = self._no_rendering
        setting.fixed_delta_seconds = self._fixed_time
        self.world.apply_settings(setting)

    def _set_traffic_manager(self):
        """
        Enable traffic manager so that auto vehicles follow traffic rules and speed settings
        """
        self._traffic_manager = self.client.get_trafficmanager()
        self._traffic_manager.set_synchronous_mode(True)

    def _spawn_ego_vehicle(self, spawn_point=None):
        ego_cfg = self._vehicles.ego
        ego_bp = self.bp_lib.find(ego_cfg.model)
        ego_bp.set_attribute("color", ego_cfg.color)
        # if spawn_point is None:
        #     ego_transform = self._get_transform_from_cfg(ego_cfg.spawn_point)
        # else:
        #     ego_transform = self._get_transform_from_cfg(spawn_point)
        ego_transform = self._get_transform_from_cfg(self.start_point)
        if self._route:
            point = self._route['start']
            ego_transform = carla.Transform(carla.Location(x=point['x'], y=point['y'], z=point['z'] + .3),
                carla.Rotation(yaw=point['yaw']))
        ego_vehicle = self.world.spawn_actor(ego_bp, ego_transform)
        self._actors.append(ego_vehicle)
        self.ego_vehicle = ego_vehicle
    
    def _spawn_other_vehicle(self):
        if self._route and self._route.get('obstacles'):
            for obstacle in self._route['obstacles']:
                blueprint = self.bp_lib.find(obstacle['model'])
                blueprint.set_attribute('color', obstacle['color'])
                transform = carla.Transform(carla.Location(x=obstacle['x'], y=obstacle['y'], z=obstacle['z']),
                    carla.Rotation(yaw=obstacle['yaw']))
                vehicle = self.world.spawn_actor(blueprint, transform)
                self._actors.append(vehicle)
                vehicle.set_simulate_physics(False)
                self.other_vehicles.append(vehicle)
            return
        for i, obs_cfg in enumerate(self._vehicles.get("obstacles", [])):
            obs_bp = self.bp_lib.find(obs_cfg.model)
            obs_bp.set_attribute("color", obs_cfg.color)
            obs_transform = self._get_transform_from_cfg(obs_cfg.spawn_point)
            obs_vehicle = self.world.spawn_actor(obs_bp, obs_transform)
            self._actors.append(obs_vehicle)
            # Behavior control
            if obs_cfg.get("is_move", False):
                obs_vehicle.set_autopilot(True)
                self._traffic_manager.set_desired_speed(obs_vehicle, obs_cfg.speed)
                self._traffic_manager.ignore_lights_percentage(obs_vehicle, 100)
            self.other_vehicles.append(obs_vehicle)
            
    def _get_transform_from_cfg(self, spawn_idx):
        """
        Get transform from spawn index
        """
        transform = self.all_default_spawn[spawn_idx]
        if self._user_defined_spawn_points:
            # Adjust z-axis to avoid collision with ground
                location = transform.location
                rotation = transform.rotation
                transform = carla.Transform(carla.Location(location.x, location.y, location.z+0.3), rotation)
        return transform

    def scene_snapshot(self):
        """Actual spawned geometry, sufficient to redraw every static obstacle."""
        obstacles = []
        for actor, specification in zip(self.other_vehicles, (self._route or {}).get('obstacles', [])):
            transform, box = actor.get_transform(), actor.bounding_box
            obstacles.append({**specification, 'actor_id': actor.id,
                'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z,
                'yaw': transform.rotation.yaw,
                'extent_x_m': box.extent.x, 'extent_y_m': box.extent.y, 'extent_z_m': box.extent.z,
                'box_offset_x_m': box.location.x, 'box_offset_y_m': box.location.y})
        return {'town': self.town_map.name, 'weather': (self._route or {}).get('weather'),
                'route': self._route, 'obstacles': obstacles}

    def tick(self):
        spectator = self.world.get_spectator()
        location = self.ego_vehicle.get_location()
        rotation = self.ego_vehicle.get_transform().rotation
        x, y, z = location.x, location.y, location.z
        transform = carla.Transform(carla.Location(x, y, z + 50), carla.Rotation(pitch=-90))
        spectator.set_transform(transform)

        self.last_tick_start_us = time.monotonic_ns() // 1000
        self.last_frame = self.world.tick()
        self.last_tick_return_us = time.monotonic_ns() // 1000
        self.snapshot = self.world.get_snapshot()

        return SimulatorObservation(
            ego=get_vehicle_state(self.ego_vehicle),
            neighbors=[get_vehicle_state(v) for v in self.other_vehicles],
            image=self.fps_image
        )

    def _build_topology(self):
        """
        Build a refined topology for global path planning.
        """
        topology = []
        for seg in self.town_map.get_topology():
            w1 = seg[0] 
            w2 = seg[1]  
            new_seg = dict()
            new_seg["entry"] = w1
            new_seg["exit"] = w2
            new_seg["path"] = []
            w1_loc = w1.transform.location 
            if w1_loc.distance(w2.transform.location) > self._sampling_resolution:
                new_waypoint = w1.next(self._sampling_resolution)[0]
                while new_waypoint.transform.location.distance(w2.transform.location) > self._sampling_resolution:
                    new_seg["path"].append(new_waypoint)
                    new_waypoint = new_waypoint.next(self._sampling_resolution)[0]
            else:
                pass
            topology.append(new_seg)
        return topology

    def generate_collision_sensor(self):
        collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self._actors.append(self.collision_sensor)
        self.collision_sensor.listen(lambda event: self._get_collision_hist(event))

    def _get_collision_hist(self, event):
        """
        Record collision intensity
        """
        impulse = event.normal_impulse
        intensity = np.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.collision_hist.append({'frame': event.frame, 'intensity': float(intensity),
                                    'other_actor': event.other_actor.type_id})
        
    def get_map_info(self):
        if self._route:
            start, end = self._route['start'], self._route['end']
            return self.town_map, self.topology, carla.Location(x=start['x'], y=start['y'], z=start['z']), carla.Location(x=end['x'], y=end['y'], z=end['z'])
        start = self.all_default_spawn[self.start_point].location
        end = self.all_default_spawn[self.end_point].location
        return self.town_map, self.topology, start, end
    
    def destroy_actors(self):
        errors = []
        for actor in reversed(self._actors):
            try:
                if actor.is_alive:
                    if isinstance(actor, carla.Sensor) and actor.is_listening:
                        actor.stop()
                    actor.destroy()
            except Exception as exc:
                errors.append(str(exc))
        self._actors.clear()
        if hasattr(self, '_traffic_manager'):
            self._traffic_manager.set_synchronous_mode(False)
        if hasattr(self, '_original_settings'):
            self.world.apply_settings(self._original_settings)
        if hasattr(self, '_original_weather'):
            self.world.set_weather(self._original_weather)
        if errors:
            raise RuntimeError('; '.join(errors))

    def control_vehicle(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=0):
        command = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse, manual_gear_shift, gear)
        self.ego_vehicle.apply_control(command)
