#!/usr/bin/env python3
"""Catch runaway simulation motion using independent physics-stamped samples."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/simulation_motion_guard.py'
SPEC = importlib.util.spec_from_file_location('simulation_motion_guard', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PhysicsMotionGuard = MODULE.PhysicsMotionGuard


class SimulationMotionGuardTests(unittest.TestCase):
    def guard(self):
        return PhysicsMotionGuard(SimpleNamespace(
            max_horizontal_speed=0.35, overspeed_duration=0.25,
            reverse_speed_threshold=0.05, reverse_duration=1.0,
            truth_topic='/ground_truth/odom'))

    def observe(self, guard, stamp, vx=0.0, vy=0.0, yaw=0.0,
                mode='MPC_TROT', armed=True):
        guard.observe(stamp, {'vx': vx, 'vy': vy, 'yaw': yaw}, mode, armed)

    def test_overspeed_uses_physics_stamps_and_rejects_early(self):
        guard = self.guard()
        self.observe(guard, 1.0, vx=0.4)
        for _ in range(100):
            self.observe(guard, 1.0, vx=0.4)
        self.observe(guard, 1.249, vx=0.4)
        with self.assertRaisesRegex(RuntimeError, 'horizontal speed'):
            self.observe(guard, 1.251, vx=0.4)
        self.assertAlmostEqual(guard.summary['failure']['truth_stamp_sim_s'], 1.251)

    def test_transient_recovery_resets_overspeed_dwell(self):
        guard = self.guard()
        self.observe(guard, 1.0, vx=0.4)
        self.observe(guard, 1.2, vx=0.1)
        self.observe(guard, 1.3, vx=0.4)
        self.observe(guard, 1.5, vx=0.4)
        self.assertIsNone(guard.summary['failure'])

    def test_forward_at_yaw90_uses_true_y_axis(self):
        guard = self.guard()
        guard.set_command(0.05, 0.0)
        self.observe(guard, 1.0, vy=0.1, yaw=math.pi / 2)
        self.observe(guard, 3.0, vy=0.1, yaw=math.pi / 2)
        self.assertAlmostEqual(guard.summary['min_command_projected_speed_mps'], 0.1)

    def test_lateral_at_yaw90_rejects_reversed_world_x(self):
        guard = self.guard()
        guard.set_command(0.0, 0.05)
        self.observe(guard, 1.0, vx=0.1, yaw=math.pi / 2)
        self.observe(guard, 1.99, vx=0.1, yaw=math.pi / 2)
        with self.assertRaisesRegex(RuntimeError, 'opposed'):
            self.observe(guard, 2.01, vx=0.1, yaw=math.pi / 2)

    def test_correct_backward_motion_is_accepted(self):
        guard = self.guard()
        guard.set_command(-0.05, 0.0)
        self.observe(guard, 1.0, vx=-0.1)
        self.observe(guard, 3.0, vx=-0.1)
        self.assertAlmostEqual(guard.summary['min_command_projected_speed_mps'], 0.1)

    def test_stop_or_direction_change_clears_wrong_way_dwell(self):
        guard = self.guard()
        guard.set_command(0.05, 0.0)
        self.observe(guard, 1.0, vx=-0.1)
        guard.set_command(0.0, 0.0)
        self.observe(guard, 2.1, vx=-0.1)
        guard.set_command(-0.05, 0.0)
        self.observe(guard, 2.2, vx=0.1)
        self.observe(guard, 3.19, vx=0.1)
        self.assertIsNone(guard.summary['failure'])

    def test_passive_and_preflight_do_not_trip_active_motion_guard(self):
        guard = self.guard()
        self.observe(guard, 1.0, vx=1.0, armed=False)
        self.observe(guard, 2.0, vx=1.0, mode='PASSIVE')
        self.observe(guard, 3.0, vx=1.0, mode='STAND_UP')
        self.observe(guard, 4.0, vx=0.1)
        self.assertIsNone(guard.summary['failure'])

    def test_phase_records_peak_before_raising(self):
        guard = self.guard()
        phase = {}
        guard.begin_phase(phase)
        self.observe(guard, 1.0, vx=0.5)
        with self.assertRaises(RuntimeError):
            self.observe(guard, 1.3, vx=0.6)
        self.assertAlmostEqual(phase['motion_safety']['max_horizontal_speed_mps'], 0.6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
