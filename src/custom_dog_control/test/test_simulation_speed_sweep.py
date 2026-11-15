#!/usr/bin/env python3
"""Pure numeric checks for independent simulation speed measurements.

Importing the script does not initialize rclpy, create a Node, publish commands
or access hardware. No simulator is required for these tests.
"""

import argparse
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from simulation_speed_sweep_test import body_velocity, parse_levels, weighted_mean


def quaternion(roll, pitch, yaw, scale=1.0):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return SimpleNamespace(w=scale*(cr*cp*cy+sr*sp*sy),
                           x=scale*(sr*cp*cy-cr*sp*sy),
                           y=scale*(cr*sp*cy+sr*cp*sy),
                           z=scale*(cr*cp*sy-sr*sp*cy))


def rotate_body_to_world(vector, roll, pitch, yaw):
    """Independent elementary-axis rotations, without a quaternion matrix."""
    x, y, z = vector
    y, z = math.cos(roll)*y-math.sin(roll)*z, math.sin(roll)*y+math.cos(roll)*z
    x, z = math.cos(pitch)*x+math.sin(pitch)*z, -math.sin(pitch)*x+math.cos(pitch)*z
    x, y = math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y
    return x, y, z


class SpeedSweepMeasurements(unittest.TestCase):
    def assert_vector_close(self, actual, expected):
        for index, (left, right) in enumerate(zip(actual, expected)):
            with self.subTest(component=index):
                self.assertAlmostEqual(left, right, places=12)

    def test_world_y_maps_to_forward_at_quarter_turn(self):
        actual = body_velocity(quaternion(0, 0, math.pi/2), (0, 2, .3))
        self.assert_vector_close(actual, (2, 0, .3))

    def test_full_roll_pitch_yaw_round_trip(self):
        for angles in ((.4, -.7, 1.2), (-.8, .5, -2.4), (1.1, 1.0, .2)):
            for vector in ((1.2, -.6, .3), (-.4, .8, -1.3)):
                with self.subTest(angles=angles, velocity=vector):
                    world = rotate_body_to_world(vector, *angles)
                    self.assert_vector_close(body_velocity(quaternion(*angles), world), vector)

    def test_quaternion_scale_and_sign_do_not_change_velocity(self):
        angles, vector = (.3, -.5, 1.8), (.5, -.4, .2)
        world = rotate_body_to_world(vector, *angles)
        for scale in (-1.0, .2, 4.0):
            with self.subTest(scale=scale):
                self.assert_vector_close(body_velocity(quaternion(*angles, scale), world), vector)

    def test_nonuniform_time_weighted_mean_and_rmse(self):
        samples = [{"stamp": 0., "speed": 0.},
                   {"stamp": 1., "speed": 2.},
                   {"stamp": 3., "speed": 2.}]
        # Trapezoid areas are 1 and 4; samples must not receive equal weight.
        self.assertAlmostEqual(weighted_mean(samples, "speed"), 5/3)
        # Error about target=2: squared-error areas are 2 and 0.
        self.assertAlmostEqual(weighted_mean(samples, "speed", 2), math.sqrt(2/3))

    def test_repeated_callbacks_do_not_reweight_time_integrals(self):
        samples = [{"stamp": 10., "speed": -.2},
                   {"stamp": 10.1, "speed": .4},
                   {"stamp": 10.8, "speed": .7},
                   {"stamp": 12., "speed": .5}]
        repeated = [dict(sample) for sample, count in zip(samples, (20, 1, 50, 3))
                    for _ in range(count)]
        self.assertAlmostEqual(weighted_mean(samples, "speed"),
                               weighted_mean(repeated, "speed"))
        self.assertAlmostEqual(weighted_mean(samples, "speed", .5),
                               weighted_mean(repeated, "speed", .5))

    def test_no_elapsed_truth_time_is_rejected(self):
        for samples in ([], [{"stamp": 1., "speed": 2.}],
                        [{"stamp": 1., "speed": 2.}]*5):
            with self.subTest(samples=samples):
                with self.assertRaises(RuntimeError):
                    weighted_mean(samples, "speed")

    def test_positive_sorted_levels_are_preserved(self):
        self.assertEqual(parse_levels(" 0.1, .3, 0.5 "), [.1, .3, .5])

    def test_invalid_levels_are_rejected(self):
        for value in ("", "nan", "inf", "-inf", "0", "-0.1", "bad", "0.2,0.1",
                      "0.1,0.1", "0.1,nan", "0.1,0", "0.1,"):
            with self.subTest(levels=value):
                with self.assertRaises(argparse.ArgumentTypeError):
                    parse_levels(value)


if __name__ == "__main__":
    unittest.main()
