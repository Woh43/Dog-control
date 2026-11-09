#!/usr/bin/env python3
"""Exercise the ROS controller against a running Gazebo Fortress physics world.

Requires /clock, independent simulator odometry, IMU, joint states and controller
diagnostics. It never opens a serial device. Run with fortress.launch.py; an
offline optimizer smoke test or synthetic joint source is not this acceptance.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from simulation_motion_guard import PhysicsMotionGuard, add_motion_safety_arguments

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Joy


JOINTS = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)


def quaternion_rpy(q):
    values = (q.w, q.x, q.y, q.z)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("simulator odometry has a non-finite quaternion")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6:
        raise RuntimeError("simulator odometry has a zero quaternion")
    w, x, y, z = (value / norm for value in values)
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def angle_delta(final, initial):
    return math.atan2(math.sin(final - initial), math.cos(final - initial))


class FortressAcceptance(Node):
    def __init__(self, args):
        super().__init__(
            "custom_dog_fortress_acceptance",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args = args
        self.latest = {}
        self.mode = "NO_DIAGNOSTICS"
        self.status_message = ""
        self.hardware_id = ""
        self.diag_sequence = 0
        self.last_diag_wall = 0.0
        self.truth = None
        self.last_truth_wall = 0.0
        self.joints = {}
        self.last_joints_wall = 0.0
        self.imu_valid = False
        self.last_imu_wall = 0.0
        self.simulation_verified = False
        self.motion_guard = PhysicsMotionGuard(args)
        self.mode_history = []
        self.phases = []
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_publisher = self.create_publisher(Joy, "/joy", 10)
        self.create_subscription(
            DiagnosticArray, "/nmpc_wbc_controller/diagnostics",
            self.on_diagnostics, 10,
        )
        self.create_subscription(
            Odometry, args.truth_topic, self.on_truth, qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joints, qos_profile_sensor_data,
        )
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != "custom_dog_control/nmpc_wbc":
                continue
            self.hardware_id = status.hardware_id
            if self.hardware_id == "custom_dog_rs485":
                raise RuntimeError("real-hardware diagnostics detected; refusing simulation commands")
            self.latest = {item.key: item.value for item in status.values}
            self.status_message = status.message
            self.mode = self.latest.get("mode", status.message)
            self.last_diag_wall = time.monotonic()
            self.diag_sequence += 1
            if not self.mode_history or self.mode_history[-1]["mode"] != self.mode:
                self.mode_history.append({"sim_seconds": self.seconds(), "mode": self.mode})

    def on_truth(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        roll, pitch, yaw = quaternion_rpy(pose.orientation)
        values = (pose.position.x, pose.position.y, pose.position.z,
                  twist.linear.x, twist.linear.y, twist.linear.z, roll, pitch, yaw)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("independent simulator odometry is not finite")
        self.truth = dict(zip(("x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"), values))
        self.last_truth_wall = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.motion_guard.observe(stamp, self.truth, self.mode, self.simulation_verified)

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))
        self.last_joints_wall = time.monotonic()

    def on_imu(self, message):
        q = message.orientation
        values = (q.w, q.x, q.y, q.z,
                  message.angular_velocity.x, message.angular_velocity.y,
                  message.angular_velocity.z, message.linear_acceleration.x,
                  message.linear_acceleration.y, message.linear_acceleration.z)
        self.imu_valid = (
            all(math.isfinite(value) for value in values)
            and sum(value * value for value in values[:4]) > 1e-12
            and message.orientation_covariance[0] != -1.0
        )
        self.last_imu_wall = time.monotonic()

    def publish(self, speed=0.0, button=None):
        if not self.simulation_verified or self.hardware_id != "gazebo":
            raise RuntimeError("simulation inputs have not passed preflight")
        if self.args.legacy_joy:
            self.publish_legacy_joy(speed, 0.0, 0.0, button)
            return
        message = Twist()
        message.linear.x = speed
        self.motion_guard.set_command(speed, 0.0)
        self.velocity_publisher.publish(message)
        if button is not None:
            joy = Joy()
            joy.axes = [0.0] * 6
            joy.buttons = [0] * 10
            joy.buttons[button] = 1
            self.joy_publisher.publish(joy)

    def publish_legacy_joy(self, vx, vy, yaw, button):
        # Exercise the original event2joy axes through the production callback;
        # no /cmd_vel is published in this test mode. These limits match the
        # shared initial profile in controllers.yaml.
        normalized = (-yaw / 0.20, -vx / 0.05, -vy / 0.05)
        if any(abs(value) > 1.000001 for value in normalized):
            raise RuntimeError("legacy Joy test exceeds the shared low-speed profile")
        joy = Joy()
        joy.axes = [normalized[0], normalized[1], -1.0, normalized[2], 0.0, -1.0]
        joy.buttons = [0] * 10
        if button is not None:
            joy.buttons[button] = 1
            if button != 7:
                joy.axes[2] = 1.0  # Legacy LT plus A/B/X/Y.
        self.motion_guard.set_command(vx, vy)
        self.joy_publisher.publish(joy)

    def check_telemetry(self, active=False):
        now = time.monotonic()
        for label, stamp in (
            ("diagnostics", self.last_diag_wall), ("simulator odometry", self.last_truth_wall),
            ("joint states", self.last_joints_wall), ("IMU", self.last_imu_wall),
        ):
            if now - stamp > self.args.telemetry_timeout:
                raise RuntimeError(f"{label} stopped updating")
        if self.mode == "FAULT":
            raise RuntimeError("controller entered FAULT: " + self.status_message + "; NMPC: " + self.latest.get("mpc_last_error", ""))
        if not self.imu_valid:
            raise RuntimeError("simulator IMU is invalid")
        if not all(name in self.joints and math.isfinite(self.joints[name]) for name in JOINTS):
            raise RuntimeError("joint feedback must contain 12 finite URDF joint angles")
        if active:
            if not self.args.min_height <= self.truth["z"] <= self.args.max_height:
                raise RuntimeError(f"standing/walking body height out of bounds: {self.truth['z']:.3f} m")
            if max(abs(self.truth["roll"]), abs(self.truth["pitch"])) > self.args.max_tilt:
                raise RuntimeError("simulated body roll/pitch exceeded the acceptance limit")

    def wait_ready(self):
        deadline = time.monotonic() + self.args.startup_timeout
        first_clock = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mode == "FAULT":
                raise RuntimeError("controller faulted before simulation startup: " + self.status_message)
            if self.hardware_id != "gazebo" or self.mode != "PASSIVE" or self.truth is None:
                continue
            if self.seconds() <= 0.0:
                continue
            if first_clock is None:
                first_clock = self.seconds()
            if self.seconds() - first_clock < 0.1:
                continue
            try:
                self.check_telemetry()
            except RuntimeError:
                continue
            if self.latest.get("nmpc_backend") != "legged::LeggedInterface":
                raise RuntimeError("unexpected NMPC backend")
            if self.latest.get("wbc_backend") != "legged::WeightedWbc":
                raise RuntimeError("unexpected WBC backend")
            self.simulation_verified = True
            print("PASSIVE ready: advancing simulation clock, physics odometry, IMU and 12 joints", flush=True)
            return
        raise RuntimeError("simulation startup timed out before PASSIVE with complete telemetry")

    def wait_mode(self, expected, sim_timeout, speed=0.0, button=None):
        start_sim, start_wall = self.seconds(), time.monotonic()
        while rclpy.ok():
            self.publish(speed, button)
            rclpy.spin_once(self, timeout_sec=0.03)
            self.check_telemetry()
            # When already standing, one Joy publication can be overwritten
            # by the following neutral snapshot before a controller tick.
            # Hold A/LT+A until diagnostics acknowledge disarming, as a person
            # holding the button would, rather than accepting the old mode.
            request_acknowledged = (button != 0 or expected != "MPC_STANCE" or
                                    self.latest.get("trot_enabled") == "false")
            if self.mode == expected and request_acknowledged:
                return
            if self.seconds() - start_sim > sim_timeout:
                raise RuntimeError(f"did not enter {expected} in {sim_timeout} simulation seconds (mode {self.mode})")
            if time.monotonic() - start_wall > max(30.0, sim_timeout * self.args.wall_factor):
                raise RuntimeError(f"wall-clock timeout waiting for {expected}; physics may be stalled")

    def hold(self, name, duration, mode, speed=0.0, require_disarmed=False):
        start_sim, start_wall = self.seconds(), time.monotonic()
        start_pose = dict(self.truth)
        result = {
            "name": name, "command_vx_mps": speed, "required_mode": mode,
            "duration_sim_s": duration, "diagnostic_samples": 0,
            "invalid_wbc_samples": 0, "max_abs_roll_rad": 0.0,
            "max_abs_pitch_rad": 0.0, "min_height_m": None,
            "max_height_m": None, "policy_modes": [],
        }
        self.phases.append(result)
        self.motion_guard.begin_phase(result)
        seen_sequence, policy_modes, next_print = -1, set(), start_wall
        while rclpy.ok() and self.seconds() - start_sim < duration:
            self.publish(speed)
            rclpy.spin_once(self, timeout_sec=0.03)
            self.check_telemetry(active=True)
            if self.mode != mode:
                raise RuntimeError(f"{name}: unexpected mode {self.mode}, expected {mode}")
            elapsed = self.seconds() - start_sim
            if elapsed < -0.01:
                raise RuntimeError("simulation clock moved backwards")
            if time.monotonic() - start_wall > max(30.0, duration * self.args.wall_factor):
                raise RuntimeError(f"{name}: physics failed to advance in the wall-clock budget")
            if self.diag_sequence != seen_sequence:
                seen_sequence = self.diag_sequence
                result["diagnostic_samples"] += 1
                if elapsed > 0.4:
                    result["invalid_wbc_samples"] += self.latest.get("wbc_valid") != "true"
                    if require_disarmed and self.latest.get("trot_enabled") != "false":
                        raise RuntimeError("explicit A/STANCE request did not keep Trot disarmed")
                    if mode == "MPC_STANCE" and self.latest.get("policy_mode") != "15":
                        raise RuntimeError(name + ": standing mode still uses a non-stance contact plan")
                policy_modes.add(int(self.latest.get("policy_mode", "-1")))
                for axis in ("roll", "pitch"):
                    key = "max_abs_" + axis + "_rad"
                    result[key] = max(result[key], abs(self.truth[axis]))
                for key, combine in (("min_height_m", min), ("max_height_m", max)):
                    previous = result[key]
                    result[key] = self.truth["z"] if previous is None else combine(previous, self.truth["z"])
            if time.monotonic() >= next_print:
                print(f"{name}: sim={elapsed:.1f}s {self.mode} xyz=({self.truth['x']:.3f},{self.truth['y']:.3f},{self.truth['z']:.3f})", flush=True)
                next_print = time.monotonic() + 2.0
        dx, dy = self.truth["x"] - start_pose["x"], self.truth["y"] - start_pose["y"]
        yaw = start_pose["yaw"]
        result.update({
            "elapsed_sim_s": self.seconds() - start_sim,
            "elapsed_wall_s": time.monotonic() - start_wall,
            "forward_displacement_m": math.cos(yaw) * dx + math.sin(yaw) * dy,
            "lateral_displacement_m": -math.sin(yaw) * dx + math.cos(yaw) * dy,
            "yaw_drift_rad": angle_delta(self.truth["yaw"], yaw),
            "final_horizontal_speed_mps": math.hypot(self.truth["vx"], self.truth["vy"]),
            "policy_modes": sorted(policy_modes),
        })
        if result["diagnostic_samples"] < 3:
            raise RuntimeError(name + ": insufficient fresh diagnostic samples")
        if result["invalid_wbc_samples"]:
            raise RuntimeError(name + ": WBC produced invalid outputs")
        if mode == "MPC_TROT":
            if result["forward_displacement_m"] < max(0.08, speed * duration * 0.4):
                raise RuntimeError(name + ": commanded forward motion did not produce sufficient physics displacement")
            if not {6, 9}.issubset(result["policy_modes"]):
                raise RuntimeError(name + ": both diagonal Trot contact plans were not observed")
            if abs(result["lateral_displacement_m"]) > 0.30 or abs(result["yaw_drift_rad"]) > 0.35:
                raise RuntimeError(name + ": excessive lateral/yaw drift")
        elif result["final_horizontal_speed_mps"] > 0.10:
            raise RuntimeError(name + ": body did not settle after stopping")
        print("PHASE_PASS " + name, flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--legacy-joy", action="store_true",
                        help="Use qr_ws Joy axes and LT combinations instead of /cmd_vel")
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--startup-timeout", type=float, default=900.0)
    parser.add_argument("--wall-factor", type=float, default=25.0)
    parser.add_argument("--telemetry-timeout", type=float, default=5.0)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--walk-duration", type=float, default=10.0)
    parser.add_argument("--resume-duration", type=float, default=6.0)
    parser.add_argument("--min-height", type=float, default=0.18)
    parser.add_argument("--max-height", type=float, default=0.40)
    parser.add_argument("--max-tilt", type=float, default=0.35)
    add_motion_safety_arguments(parser)
    args = parser.parse_args()
    positive = (args.startup_timeout, args.wall_factor, args.telemetry_timeout,
                args.walk_duration, args.resume_duration, args.min_height,
                args.max_height, args.max_tilt, args.max_horizontal_speed,
                args.overspeed_duration, args.reverse_speed_threshold, args.reverse_duration)
    if not all(math.isfinite(value) and value > 0 for value in positive) or args.min_height >= args.max_height:
        parser.error("timeouts/limits must be finite positive values, with min-height < max-height")
    if not 0.04 < args.speed <= 0.15 or min(args.walk_duration, args.resume_duration) < 2.0:
        parser.error("speed must be in (0.04,0.15] and movement phases at least 2 simulation seconds")
    rclpy.init(args=[])
    node = FortressAcceptance(args)
    report = {
        "passed": False, "backend": "Gazebo Fortress physics",
        "hardware_io": False, "synthetic_static_observation": False,
        "truth_topic": args.truth_topic, "simulation_preflight_passed": False,
        "settings": vars(args) | {"output": str(args.output)},
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_scope": "Simulation joints already use URDF coordinates; real encoder/mechanical calibration is not certified.",
    }
    code = 1
    try:
        node.wait_ready()
        report["simulation_preflight_passed"] = True
        if args.legacy_joy:
            start = node.seconds()
            while node.seconds() - start < 0.6:
                node.publish(0.0, button=7)
                rclpy.spin_once(node, timeout_sec=0.03)
                node.check_telemetry()
                if node.mode not in ("PASSIVE", "CALIBRATION"):
                    raise RuntimeError("legacy START must not request standing")
            if node.latest.get("requires_recalibration") != "false":
                raise RuntimeError("legacy START did not finish the calibration gate")
            node.wait_mode("MPC_STANCE", 20.0, button=0)
        else:
            node.wait_mode("MPC_STANCE", 20.0, button=7)
        if not any(item["mode"] == "STAND_UP" for item in node.mode_history):
            raise RuntimeError("START skipped the expected position-interpolated STAND_UP phase")
        node.hold("initial_stance", 3.0, "MPC_STANCE")
        node.wait_mode("MPC_TROT", 6.0, speed=args.speed,
                       button=2 if args.legacy_joy else None)
        node.hold("first_forward_trot", args.walk_duration, "MPC_TROT", speed=args.speed)
        node.wait_mode("MPC_STANCE", 8.0)
        node.hold("automatic_zero_speed_stop", 2.0, "MPC_STANCE")
        if node.latest.get("trot_enabled") != "true":
            raise RuntimeError("zero-speed stopping cleared the locomotion arm flag")
        node.wait_mode("MPC_TROT", 6.0, speed=args.speed)
        node.hold("resume_without_rearming", args.resume_duration, "MPC_TROT", speed=args.speed)
        node.wait_mode("MPC_STANCE", 8.0, speed=args.speed, button=0)
        # Keep cmd_vel nonzero after A has expired. A broken state transition
        # would arm itself again and return to Trot after the entry dwell.
        node.hold("explicit_A_stays_in_stance", 3.0, "MPC_STANCE", speed=args.speed, require_disarmed=True)
        node.hold("final_zero_speed_stance", 2.0, "MPC_STANCE")
        report["passed"] = True
        code = 0
        print("PASS: physics stand-up, forward Trot, automatic stop/resume and explicit A disarm", flush=True)
    except (RuntimeError, ValueError) as error:
        report["error"] = str(error)
        print("FAIL: " + str(error), file=sys.stderr, flush=True)
    finally:
        report["phases"] = node.phases
        report["mode_history"] = node.mode_history
        report["final_mode"] = node.mode
        report["final_status_message"] = node.status_message
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
