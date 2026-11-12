"""Regression contracts for causal selection and the shared PPO/fusion input."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'vendor/HOCD'))
from bridge.control_client import SteeringClient


class SteeringContract(unittest.TestCase):
    def setUp(self):
        self.client = SteeringClient('A', 'tirex', bind=('127.0.0.1', 0))
        self.observation = {'version': 1, 'trial_id': 'A', 'kind': 'observation', 'sequence': 1,
            'source_time_us': 1_000_000, 'native_source_time_us': 995_000,
            'last_observation': -.5, 'mapped_source_time_us': 1_000_000,
            'timing_valid': True, 'context_ready': True}
        self.forecast = {'version': 1, 'trial_id': 'A', 'kind': 'trajectory', 'sequence': 2,
            'forecast_id': 7, 's_f': 1_000_000, 'anchor_value': 0., 'sample_period_us': 25_000,
            'median': [1., 2.], 't_f': 1_000_000., 'r': 1., 'valid': True, 'compute_us': 100_000}

    def tearDown(self):
        self.client.close()

    def load(self):
        self.client.ingest(self.observation)
        self.client.ingest(self.forecast)

    def test_raw_input_cannot_bypass_the_received_chain(self):
        self.client.send_raw(3., 1_000_000)
        selected = self.client.select(1_010_000)
        self.assertEqual(selected.mode, 'unavailable')
        self.assertEqual(selected.value, 0.)

    def test_each_control_tick_interpolates_at_its_own_planned_time(self):
        self.load()
        self.assertAlmostEqual(self.client.select(1_010_000).value, .4)
        self.assertAlmostEqual(self.client.select(1_020_000).value, .8)
        self.assertAlmostEqual(self.client.select(1_050_000).value, 2.)

    def test_no_extrapolation_after_the_horizon_or_before_anchor(self):
        self.load()
        for timestamp in (999_999, 1_050_001):
            value = self.client.select(timestamp)
            self.assertEqual((value.mode, value.value), ('hold', -.5))

    def test_new_observation_never_reanchors_an_old_forecast(self):
        self.load()
        self.client.ingest({**self.observation, 'sequence': 3, 'source_time_us': 1_025_000, 'last_observation': 99.})
        self.assertAlmostEqual(self.client.select(1_037_500).value, 1.5)
        self.assertEqual(self.client.trajectory['s_f'], 1_000_000)

    def test_mapping_update_can_shift_time_but_cannot_change_source_anchor(self):
        self.load()
        self.client.ingest({**self.forecast, 'sequence': 4, 't_f': 1_001_000., 'r': 1.0001})
        value = self.client.select(1_013_500)
        self.assertAlmostEqual(value.value, .5 / 1.0001)
        with self.assertRaises(ValueError):
            self.client.ingest({**self.forecast, 'sequence': 5, 's_f': 1_025_000})
        self.assertEqual(self.client.trajectory['s_f'], 1_000_000)

    def test_cross_trial_and_stale_messages_do_not_replace_cache(self):
        self.load()
        for msg in ({**self.forecast, 'sequence': 3, 'trial_id': 'B'},
                    {**self.forecast, 'sequence': 1},
                    {**self.forecast, 'sequence': 4, 'forecast_id': 6}):
            with self.assertRaises(ValueError):
                self.client.ingest(msg)
        self.assertEqual(self.client.trajectory['forecast_id'], 7)

    def test_invalid_mapping_or_nonfinite_trajectory_falls_back_safely(self):
        self.load()
        self.client.ingest({**self.forecast, 'sequence': 4, 'valid': False, 't_f': None})
        self.assertEqual(self.client.select(1_012_500).value, -.5)
        with self.assertRaises(ValueError):
            self.client.ingest({**self.forecast, 'sequence': 5, 'median': [float('nan')]})
        self.assertFalse(self.client.trajectory['valid'])

    def test_hold_uses_the_same_delayed_observation_even_with_a_forecast(self):
        self.load()
        self.client.condition = 'hold'
        self.assertEqual(self.client.select(1_012_500).value, -.5)


class SharedControlContract(unittest.TestCase):
    def test_empty_planner_candidate_stops_before_dereferencing_path(self):
        from planner.vehicle_planner import VehiclePlan
        planner = VehiclePlan.__new__(VehiclePlan)
        planner.detect_obstacle = Mock(return_value=[])
        planner._frenet_optimal_planning = Mock(return_value=None)
        with self.assertRaisesRegex(RuntimeError, 'no_candidate_path'):
            planner.run_step(None)

    def test_ppo_and_fusion_use_exactly_the_same_delayed_human_input(self):
        import os
        from omegaconf import OmegaConf
        from controller.share import SharedControl
        from utils.data import LatDebugInfo
        previous = Path.cwd()
        os.chdir(ROOT / 'vendor/HOCD')
        try:
            config = OmegaConf.load('utils/config.yaml')
            config.controller.lateral.G29.diagnostic_input = True
            control = SharedControl(config)
            control.vehicle_control.control = Mock(return_value=(.2, LatDebugInfo()))
            observed = []
            control._load_action = lambda state: observed.append(state.copy()) or .25
            state = Mock(ay=.03)
            steer, info = control.control([], state, (.1, 0., .04, 0.), 0, human_steer=-.7)
            self.assertAlmostEqual(observed[0][4], -.7)
            self.assertAlmostEqual(steer, .25 * .2 + .75 * -.7)
            self.assertAlmostEqual(info.steer_driver, -.7)
            with self.assertRaises(ValueError):
                control.control([], state, (.1, 0., .04, 0.), 0)
        finally:
            os.chdir(previous)


if __name__ == '__main__':
    unittest.main()
