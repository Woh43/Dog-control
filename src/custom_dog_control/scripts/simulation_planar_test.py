#!/usr/bin/env python3
"""Verify low-speed planar directions in a running Fortress physics world.

Start only after the robot is already in MPC_STANCE. This uses independent
physics odometry, the advancing simulation clock and the existing controller
diagnostics. It does not open serial ports or request calibration/stand-up.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import Joy

from simulation_fortress_test import (
    FortressAcceptance, add_motion_safety_arguments, angle_delta,
)


class PlanarAcceptance(FortressAcceptance):
    def publish(self, speed=0.0, button=None, *, vy=0.0, yaw=0.0):
        # Preserve the parent method's argument order for its stance helpers.
        if not self.simulation_verified or self.hardware_id != "gazebo":
            raise RuntimeError("simulation inputs have not passed preflight")
        if self.args.legacy_joy:
            self.publish_legacy_joy(speed, vy, yaw, button)
            return
        message = Twist()
        message.linear.x = float(speed)
        message.linear.y = float(vy)
        message.angular.z = float(yaw)
        self.motion_guard.set_command(speed, vy)
        self.velocity_publisher.publish(message)
        if button is not None:
            joy = Joy()
            joy.axes = [0.0] * 6
            joy.buttons = [0] * 10
            joy.buttons[button] = 1
            self.joy_publisher.publish(joy)

    def wait_standing_ready(self):
        deadline = time.monotonic() + self.args.startup_timeout
        first_clock = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mode == "FAULT":
                raise RuntimeError("controller is faulted before planar preflight")
            if self.hardware_id != "gazebo" or self.mode != "MPC_STANCE" or self.truth is None:
                continue
            if self.seconds() <= 0.0:
                continue
            if first_clock is None:
                first_clock = self.seconds()
            if self.seconds() - first_clock < 0.1:
                continue
            try:
                self.check_telemetry(active=True)
            except RuntimeError:
                continue
            if self.latest.get("nmpc_backend") != "legged::LeggedInterface":
                raise RuntimeError("unexpected NMPC backend")
            if self.latest.get("wbc_backend") != "legged::WeightedWbc":
                raise RuntimeError("unexpected WBC backend")
            if self.latest.get("policy_mode") != "15":
                continue
            self.simulation_verified = True
            print("MPC_STANCE ready: advancing /clock, physics truth, IMU and 12 joints", flush=True)
            return
        raise RuntimeError("planar preflight timed out; requires an already standing physics robot")

    def wait_trot(self, command):
        start_sim, start_wall = self.seconds(), time.monotonic()
        while rclpy.ok():
            self.publish(command[0], button=2, vy=command[1], yaw=command[2])
            rclpy.spin_once(self, timeout_sec=0.03)
            self.check_telemetry(active=True)
            if self.mode == "MPC_TROT":
                return
            if self.seconds() - start_sim > 6.0:
                raise RuntimeError("X and the low-speed command did not enter MPC_TROT")
            if time.monotonic() - start_wall > max(30.0, 6.0 * self.args.wall_factor):
                raise RuntimeError("physics stalled while waiting for Trot")

    def motion_phase(self, name, command, tested_axis, sign):
        self.wait_trot(command)
        start_sim, start_wall = self.seconds(), time.monotonic()
        start_pose = dict(self.truth)
        result = {
            "name": name, "required_mode": "MPC_TROT",
            "command": {"vx_mps": command[0], "vy_mps": command[1], "yaw_radps": command[2]},
            "duration_sim_s": self.args.phase_duration,
            "tested_axis": tested_axis, "expected_sign": sign,
            "diagnostic_samples": 0, "invalid_wbc_samples": 0,
            "max_abs_roll_rad": 0.0, "max_abs_pitch_rad": 0.0,
            "min_height_m": None, "max_height_m": None,
            "start_truth": start_pose,
        }
        self.phases.append(result)
        self.motion_guard.begin_phase(result)
        seen_sequence, policy_modes, next_print = -1, set(), start_wall
        while rclpy.ok() and self.seconds() - start_sim < self.args.phase_duration:
            self.publish(command[0], vy=command[1], yaw=command[2])
            rclpy.spin_once(self, timeout_sec=0.03)
            self.check_telemetry(active=True)
            if self.mode != "MPC_TROT":
                raise RuntimeError(f"{name}: unexpected mode {self.mode}")
            elapsed = self.seconds() - start_sim
            if elapsed < -0.01:
                raise RuntimeError("simulation clock moved backwards")
            if time.monotonic() - start_wall > max(30.0, self.args.phase_duration * self.args.wall_factor):
                raise RuntimeError(name + ": physics did not advance within the wall-time budget")
            if self.diag_sequence != seen_sequence:
                seen_sequence = self.diag_sequence
                result["diagnostic_samples"] += 1
                if elapsed > 0.4:
                    result["invalid_wbc_samples"] += self.latest.get("wbc_valid") != "true"
                policy_modes.add(int(self.latest.get("policy_mode", "-1")))
                for axis in ("roll", "pitch"):
                    key = "max_abs_" + axis + "_rad"
                    result[key] = max(result[key], abs(self.truth[axis]))
                for key, combine in (("min_height_m", min), ("max_height_m", max)):
                    previous = result[key]
                    result[key] = self.truth["z"] if previous is None else combine(previous, self.truth["z"])
            if time.monotonic() >= next_print:
                print(f"{name}: sim={elapsed:.1f}s xyz=({self.truth['x']:.3f},"
                      f"{self.truth['y']:.3f},{self.truth['z']:.3f}) yaw={self.truth['yaw']:.3f}", flush=True)
                next_print = time.monotonic() + 2.0
        dx, dy = self.truth["x"] - start_pose["x"], self.truth["y"] - start_pose["y"]
        yaw = start_pose["yaw"]
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        rotation = angle_delta(self.truth["yaw"], yaw)
        result.update({
            "elapsed_sim_s": self.seconds() - start_sim,
            "elapsed_wall_s": time.monotonic() - start_wall,
            "forward_displacement_m": forward, "lateral_displacement_m": lateral,
            "yaw_displacement_rad": rotation, "policy_modes": sorted(policy_modes),
            "final_truth": dict(self.truth),
        })
        if result["diagnostic_samples"] < 3 or result["invalid_wbc_samples"]:
            raise RuntimeError(name + ": insufficient diagnostics or invalid WBC outputs")
        if not {6, 9}.issubset(policy_modes):
            raise RuntimeError(name + ": both diagonal Trot contact modes were not observed")
        if tested_axis == "yaw":
            minimum = max(0.15, abs(command[2]) * self.args.phase_duration * 0.25)
            if sign * rotation < minimum:
                raise RuntimeError(name + ": yaw direction/displacement did not match the command")
            if math.hypot(forward, lateral) > self.args.max_turn_translation:
                raise RuntimeError(name + ": excessive translation during an in-place yaw request")
        else:
            actual = forward if tested_axis == "forward" else lateral
            target = command[0] if tested_axis == "forward" else command[1]
            minimum = max(0.04, abs(target) * self.args.phase_duration * 0.25)
            if sign * actual < minimum:
                raise RuntimeError(name + ": physics displacement direction did not match the command")
            off_axis = lateral if tested_axis == "forward" else forward
            if abs(off_axis) > self.args.max_off_axis or abs(rotation) > self.args.max_yaw_drift:
                raise RuntimeError(name + ": excessive off-axis/yaw drift")
        result["minimum_required_displacement"] = minimum
        result["passed"] = True
        print("PHASE_PASS " + name, flush=True)

    def settle(self, name):
        self.wait_mode("MPC_STANCE", 8.0)
        self.hold(name, self.args.settle_duration, "MPC_STANCE")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--legacy-joy", action="store_true",
                        help="Use qr_ws Joy axes and LT combinations instead of /cmd_vel")
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--telemetry-timeout", type=float, default=5.0)
    parser.add_argument("--wall-factor", type=float, default=25.0)
    parser.add_argument("--phase-duration", type=float, default=5.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument("--soak-duration", type=float, default=0.0,
                        help="Additional final stance simulation seconds")
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--yaw-rate", type=float, default=0.20)
    parser.add_argument("--min-height", type=float, default=0.18)
    parser.add_argument("--max-height", type=float, default=0.40)
    parser.add_argument("--max-tilt", type=float, default=0.35)
    parser.add_argument("--max-off-axis", type=float, default=0.15)
    parser.add_argument("--max-yaw-drift", type=float, default=0.30)
    parser.add_argument("--max-turn-translation", type=float, default=0.12)
    add_motion_safety_arguments(parser)
    args = parser.parse_args()
    positive = (args.startup_timeout, args.telemetry_timeout, args.wall_factor,
                args.settle_duration, args.min_height, args.max_height, args.max_tilt,
                args.max_off_axis, args.max_yaw_drift, args.max_turn_translation,
                args.max_horizontal_speed, args.overspeed_duration,
                args.reverse_speed_threshold, args.reverse_duration)
    if not all(math.isfinite(value) and value > 0 for value in positive) or args.min_height >= args.max_height:
        parser.error("timeouts and limits must be finite positive values, min-height < max-height")
    if not 0.04 < args.speed <= 0.10 or not 0.15 < args.yaw_rate <= 0.30:
        parser.error("this basic-direction test limits speed to (0.04,0.10] and yaw rate to (0.15,0.30]")
    if not 3.0 <= args.phase_duration <= 10.0:
        parser.error("phase-duration must be 3..10 simulation seconds")
    if not math.isfinite(args.soak_duration) or not 0.0 <= args.soak_duration <= 600.0:
        parser.error("soak-duration must be 0..600 simulation seconds")
    rclpy.init(args=[])
    node = PlanarAcceptance(args)
    report = {
        "passed": False, "backend": "Gazebo Fortress physics", "hardware_io": False,
        "synthetic_static_observation": False, "simulation_preflight_passed": False,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "settings": vars(args) | {"output": str(args.output)},
        "scope": "Low-speed backward, left, right and both yaw directions; starts in MPC_STANCE.",
    }
    code = 1
    try:
        node.wait_standing_ready()
        report["simulation_preflight_passed"] = True
        node.settle("initial_stance")
        cases = (
            ("backward", (-args.speed, 0.0, 0.0), "forward", -1),
            ("left", (0.0, args.speed, 0.0), "lateral", 1),
            ("right", (0.0, -args.speed, 0.0), "lateral", -1),
            ("positive_yaw", (0.0, 0.0, args.yaw_rate), "yaw", 1),
            ("negative_yaw", (0.0, 0.0, -args.yaw_rate), "yaw", -1),
        )
        for name, command, axis, sign in cases:
            node.motion_phase(name, command, axis, sign)
            node.settle("zero_speed_stance_after_" + name)
        node.wait_mode("MPC_STANCE", 8.0, button=0)
        node.hold("final_A_zero_speed_stance", args.settle_duration, "MPC_STANCE", require_disarmed=True)
        if args.soak_duration:
            node.hold("final_stance_soak", args.soak_duration, "MPC_STANCE", require_disarmed=True)
        report["passed"] = True
        code = 0
        print("PASS: basic backward, left, right and yaw directions in physical simulation", flush=True)
    except (RuntimeError, ValueError) as error:
        report["error"] = str(error)
        print("FAIL: " + str(error), file=sys.stderr, flush=True)
    finally:
        report["phases"] = node.phases
        report["mode_history"] = node.mode_history
        report["final_mode"] = node.mode
        report["final_diagnostics"] = node.latest
        report["motion_safety"] = node.motion_guard.summary
        if node.simulation_verified:
            try:
                for _ in range(5):
                    node.publish(0.0, button=0)
                    rclpy.spin_once(node, timeout_sec=0.02)
            except (RuntimeError, rclpy.exceptions.RCLError):
                pass
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
