#!/usr/bin/env python3
"""Validate real launch parameter merging without launching nodes or opening IO."""

import importlib.util
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from launch import LaunchContext
from launch.actions import LogInfo, SetLaunchConfiguration
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'dog_real_launch', PACKAGE / 'launch/real.launch.py')
REAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REAL)


class RealLaunchTimingTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='dog_launch_timing_test_')
        self.addCleanup(temporary.cleanup)
        self.log_dir = Path(temporary.name)
        self.context = LaunchContext()
        self.context.launch_configurations.update(
            {'task_file': '', 'control_frequency_hz': ''})
        self.addCleanup(patch.stopall)
        patch.object(REAL, 'launch_config', SimpleNamespace(log_dir=str(self.log_dir))).start()
        # LogInfo remains a real launch action, but its constructor must not
        # create a ROS log directory as a side effect of this offline test.
        patch('launch.logging.get_logger', return_value=Mock()).start()

    def configure(self, override=''):
        self.context.launch_configurations['control_frequency_hz'] = override
        actions = REAL._configure_controllers(
            self.context, PACKAGE / 'config/controllers.yaml',
            PACKAGE / 'config/real_controller.yaml', '/model/custom_dog.urdf')
        for action in actions:
            if isinstance(action, SetLaunchConfiguration):
                action.execute(self.context)
        runtime_file = Path(self.context.launch_configurations['real_controllers_file'])
        self.addCleanup(shutil.rmtree, runtime_file.parent, ignore_errors=True)
        self.assertEqual(runtime_file.read_text(),
                         (self.log_dir / 'real-controllers.yaml').read_text())
        message = next(action for action in actions if isinstance(action, LogInfo))
        self.message = ''.join(item.perform(self.context) for item in message.msg)
        config = yaml.safe_load(runtime_file.read_text())
        return (config['controller_manager']['ros__parameters'],
                config['nmpc_wbc_controller']['ros__parameters'])

    def test_default_inherits_frequency_and_failure_count(self):
        manager, controller = self.configure()
        self.assertEqual(manager['update_rate'], 1000)
        self.assertEqual(controller['max_consecutive_wbc_failures'], 5)
        self.assertEqual(controller['mpc_frequency_hz'], 50.0)
        self.assertNotIn('candidate', self.message)
        self.assertFalse(manager['use_sim_time'])
        self.assertEqual(controller['hardware_mode'], 'real')

    def test_250_hz_tightens_failure_budget_and_preserves_nmpc_and_gains(self):
        _, baseline = self.configure()
        manager, controller = self.configure('250')
        self.assertEqual(manager['update_rate'], 250)
        self.assertIsInstance(manager['update_rate'], int)
        self.assertEqual(controller['max_consecutive_wbc_failures'], 1)
        expected = dict(baseline, max_consecutive_wbc_failures=1)
        self.assertEqual(controller, expected)
        self.assertIn('1000 -> 250 Hz', self.message)
        self.assertIn('WBC failure cycles 5 -> 1', self.message)
        self.assertIn('NMPC 50 Hz; policy age limit 0.04 s', self.message)
        self.assertIn('candidate, requires standing validation', self.message)

    def test_original_failure_time_budget_boundary_is_allowed(self):
        manager, controller = self.configure('200')
        self.assertEqual(manager['update_rate'], 200)
        self.assertEqual(controller['max_consecutive_wbc_failures'], 1)

    def test_higher_rate_does_not_relax_failure_count(self):
        manager, controller = self.configure('2000')
        self.assertEqual(manager['update_rate'], 2000)
        self.assertEqual(controller['max_consecutive_wbc_failures'], 5)
        self.assertEqual(controller['mpc_frequency_hz'], 50.0)

    def test_equal_rate_does_not_change_failure_count(self):
        _, controller = self.configure('1000')
        self.assertEqual(controller['max_consecutive_wbc_failures'], 5)

    def test_invalid_or_too_low_rates_are_rejected(self):
        for value in ('0', '-250', 'nan', 'inf', '250.5', '250.0', 'abc', '199', '1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.configure(value)

    def test_cycle_trace_uses_this_run_directory_and_can_be_disabled(self):
        self.context.launch_configurations['cycle_trace'] = 'true'
        actions = REAL._configure_cycle_trace(self.context)
        for action in actions:
            if isinstance(action, SetLaunchConfiguration):
                action.execute(self.context)
        expected = self.log_dir / 'cycle-trace'
        self.assertTrue(expected.is_dir())
        self.assertEqual(self.context.launch_configurations['cycle_trace_directory'], str(expected))
        self.context.launch_configurations['cycle_trace'] = 'false'
        for action in REAL._configure_cycle_trace(self.context):
            if isinstance(action, SetLaunchConfiguration):
                action.execute(self.context)
        self.assertEqual(self.context.launch_configurations['cycle_trace_directory'], '')
        self.assertTrue(expected.is_dir())  # Disabling never deletes evidence.

    def test_cycle_trace_refuses_reused_path_and_invalid_boolean(self):
        expected = self.log_dir / 'cycle-trace'
        expected.symlink_to('/tmp', target_is_directory=True)
        self.context.launch_configurations['cycle_trace'] = 'true'
        for action in REAL._configure_cycle_trace(self.context):
            if isinstance(action, SetLaunchConfiguration):
                action.execute(self.context)
        self.assertEqual(self.context.launch_configurations['cycle_trace_directory'], '')
        self.context.launch_configurations['cycle_trace'] = 'maybe'
        with self.assertRaises(ValueError):
            REAL._configure_cycle_trace(self.context)


if __name__ == '__main__':
    unittest.main()
