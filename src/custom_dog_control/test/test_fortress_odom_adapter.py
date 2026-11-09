#!/usr/bin/env python3
"""Check Fortress odometry conversion against independent rigid-body kinematics."""

import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/fortress_odom_adapter.py'
SPEC = importlib.util.spec_from_file_location('fortress_odom_adapter', SCRIPT)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def rotation(roll, pitch, yaw):
    """Construct world-from-body rotation through three elementary matrices."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1., 0., 0.), (0., cr, -sr), (0., sr, cr)))
    ry = np.array(((cp, 0., sp), (0., 1., 0.), (-sp, 0., cp)))
    rz = np.array(((cy, -sy, 0.), (sy, cy, 0.), (0., 0., 1.)))
    return rz @ ry @ rx


def quaternion(roll, pitch, yaw):
    """Compose elementary unit quaternions, returning ROS XYZW order."""
    def product(a, b):
        av, aw = np.asarray(a[:3]), a[3]
        bv, bw = np.asarray(b[:3]), b[3]
        return np.r_[aw * bv + bw * av + np.cross(av, bv),
                     aw * bw - np.dot(av, bv)]

    qx = (math.sin(roll / 2), 0., 0., math.cos(roll / 2))
    qy = (0., math.sin(pitch / 2), 0., math.cos(pitch / 2))
    qz = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))
    return product(product(qz, qy), qx)


class FortressOdometryConversion(unittest.TestCase):
    def test_identity_preserves_axes_and_rates(self):
        linear, angular = ADAPTER.world_twist(
            (0., 0., 0., 1.), (1., -2., 3.), (.4, -.5, .6))
        np.testing.assert_allclose(linear, (1., -2., 3.), atol=1e-12)
        np.testing.assert_allclose(angular, (.4, -.5, .6), atol=1e-12)

    def test_yaw_quarter_turn_maps_forward_to_world_y(self):
        linear, angular = ADAPTER.world_twist(
            quaternion(0., 0., math.pi / 2), (2., 0., .3), (1., 0., .2))
        np.testing.assert_allclose(linear, (0., 2., .3), atol=1e-12)
        np.testing.assert_allclose(angular, (0., 1., .2), atol=1e-12)

    def test_random_linear_velocities_match_rotation(self):
        generator = np.random.default_rng(20261102)
        for _ in range(100):
            roll, pitch, yaw = generator.uniform((-2., -1.4, -math.pi),
                                                (2., 1.4, math.pi))
            velocity = generator.normal(size=3)
            actual, _ = ADAPTER.world_twist(quaternion(roll, pitch, yaw),
                                           velocity, (0., 0., 0.))
            np.testing.assert_allclose(actual, rotation(roll, pitch, yaw) @ velocity,
                                       rtol=1e-11, atol=1e-11)

    def test_world_angular_velocity_matches_rotation_derivative(self):
        generator = np.random.default_rng(3588)
        step = 1e-7
        for _ in range(100):
            angles = generator.uniform((-2., -1.4, -math.pi),
                                       (2., 1.4, math.pi))
            rates = generator.normal(size=3)
            r = rotation(*angles)
            derivative = (rotation(*(angles + step * rates)) -
                          rotation(*(angles - step * rates))) / (2 * step)
            skew = derivative @ r.T
            expected = np.array((skew[2, 1], skew[0, 2], skew[1, 0]))
            _, actual = ADAPTER.world_twist(quaternion(*angles), (0., 0., 0.), rates)
            np.testing.assert_allclose(actual, expected, rtol=2e-7, atol=2e-8)

    def test_quaternion_sign_and_scale_do_not_change_velocity(self):
        q = quaternion(.5, -.4, 1.2)
        expected = ADAPTER.world_twist(q, (1., 2., 3.), (.2, -.4, .6))
        for scale in (-1., .25, 3.):
            actual = ADAPTER.world_twist(q * scale, (1., 2., 3.), (.2, -.4, .6))
            np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)

    def test_invalid_inputs_are_rejected(self):
        valid = [(0., 0., 0., 1.), (1., 2., 3.), (.1, .2, .3)]
        with self.assertRaises(ValueError):
            ADAPTER.world_twist((0., 0., 0., 0.), *valid[1:])
        for group, values in enumerate(valid):
            for index in range(len(values)):
                for invalid in (float('nan'), float('inf')):
                    arguments = list(valid)
                    changed = list(values)
                    changed[index] = invalid
                    arguments[group] = changed
                    with self.subTest(group=group, index=index, invalid=invalid):
                        with self.assertRaises(ValueError):
                            ADAPTER.world_twist(*arguments)


if __name__ == '__main__':
    unittest.main()
