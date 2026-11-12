"""No raw-input bypass, no automatic lateral authority in the manual condition."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'vendor/HOCD')]
from bridge.experiment import MAPS, condition_settings, load_scene, terminal_state
from controller.delayed_manual import DelayedManualControl
from controller.vehicle_controller import VehicleControl
from utils.data import PlanInfo, VehicleParam, VehicleState


class ManualContract(unittest.TestCase):
    def setUp(self):
        self.control = DelayedManualControl(OmegaConf.create({'controller': {'lateral': {
            'G29': {'diagnostic_input': True}}}}))
        self.control.driver_control.sample = Mock(side_effect=AssertionError('Raw input bypass'))

    def test_manual_uses_received_estimate_with_zero_automatic_authority(self):
        for value in (-.7, 0., .9):
            command, info = self.control.control([], None, None, 0, human_steer=value)
            self.assertEqual(command, value)
            self.assertEqual(info.authority, 0.)
            self.assertEqual(info.steer_vehicle, 0.)
        self.control.driver_control.sample.assert_not_called()

    def test_missing_and_invalid_estimates_cannot_fall_back_to_live_wheel(self):
        with self.assertRaisesRegex(ValueError, 'bridge human_steer'):
            self.control.control([], None, None, 0)
        for value in (float('nan'), float('inf')):
            with self.assertRaisesRegex(ValueError, 'Nonfinite'):
                self.control.control([], None, None, 0, human_steer=value)

    def test_manual_retains_the_same_prediction_and_delay_as_shared_tirex(self):
        shared, manual = condition_settings('tirex'), condition_settings('manual_tirex')
        for key in ('bridge_condition', 'prediction', 'delay_us'):
            self.assertEqual(shared[key], manual[key])
        self.assertFalse(manual['shared_control'])
        self.assertTrue(condition_settings('hold')['shared_control'])
        self.assertEqual(condition_settings('hold')['delay_us'], 300_000)

    def test_complete_manual_controller_uses_common_speed_and_actuator_limits(self):
        config = OmegaConf.load(ROOT / 'configs/hocd.yaml')
        config.controller.lateral.type = 'DelayedManualControl'
        config.controller.lateral.G29.diagnostic_input = True
        config.controller.target_speed_kmh = 30
        controller = VehicleControl(config)
        self.assertIsInstance(controller.lat_control, DelayedManualControl)
        ego = VehicleState(VehicleParam(1.4, 1.4, -110000, -110000, 1800, 3500),
            0., 0., 0., 4., 0., 0., 0., 0., 0., 0., [1, 0, 0], [0, 1, 0], [0, 0, 1])
        path = [(float(x), 0., 0., 0.) for x in range(20)]
        plan = PlanInfo(path, path, [1.] * len(path), 100.)
        for value in (-2., .3, 2.):
            command = controller.run_step(Mock(ego=ego), plan, human_steer=value)
            self.assertAlmostEqual(command.lon.target_speed, 30 / 3.6)
            self.assertEqual(command.steer, float(np.clip(value, -1., 1.)))
            self.assertEqual(command.lat.authority, 0.)


class SceneContract(unittest.TestCase):
    def test_each_fixed_reference_bypasses_physical_obstacles(self):
        for town in MAPS:
            _, scene = load_scene(town)
            reference = np.array(scene['waypoints'])[:, :2]
            self.assertEqual(scene['max_seconds'], 90.)
            self.assertEqual(scene['off_route_limit_m'], 5.)
            self.assertEqual(scene['expected_scoring_seconds'], [30, 45])
            self.assertGreaterEqual(len(scene['obstacles']), 2)
            self.assertLess(np.linalg.norm(reference[0] - [scene['start']['x'], scene['start']['y']]), .01)
            for obstacle in scene['obstacles']:
                self.assertGreater(np.linalg.norm(reference - [obstacle['x'], obstacle['y']], axis=1).min(), 3.35)


class EndConditionContract(unittest.TestCase):
    def state(self, **changes):
        return terminal_state(**({'collisions': [], 'lateral_error': 0., 'progress': 120.,
            'route_length': 300., 'distance_to_end': 180., 'elapsed': 20.,
            'max_seconds': 90.} | changes))

    def test_collision_at_finish_is_a_failed_attempt(self):
        result = self.state(collisions=[{'frame': 1}], distance_to_end=1., progress=299.)
        self.assertEqual(result, {'completed': False, 'exit_reason': 'collision'})

    def test_off_route_boundary_and_ninety_second_limit(self):
        self.assertIsNone(self.state(lateral_error=5.))
        self.assertEqual(self.state(lateral_error=-5.01)['exit_reason'], 'left_route')
        self.assertIsNone(self.state(elapsed=89.99))
        self.assertEqual(self.state(elapsed=90.)['exit_reason'], 'time_limit')

    def test_finish_requires_route_progress_and_scoring_started(self):
        self.assertIsNone(self.state(distance_to_end=1., progress=10.))
        self.assertIsNone(self.state(distance_to_end=1., progress=299., elapsed=None))
        self.assertTrue(self.state(distance_to_end=1., progress=299.)['completed'])


if __name__ == '__main__':
    unittest.main()
