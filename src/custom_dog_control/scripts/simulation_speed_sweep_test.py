#!/usr/bin/env python3
"""Sweep signed Trot speeds using independent Fortress physics telemetry.

This is a simulation-only acceptance. It refuses non-Gazebo diagnostics and
requires an advancing simulation clock, truth odometry, IMU and all 12 joints.
No serial device is opened. Each failed stage is retained, commands are stopped
in finally, and faults are never reset to continue a sweep.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import Joy

from simulation_fortress_test import add_motion_safety_arguments, angle_delta
from simulation_planar_test import PlanarAcceptance


REQUEST_KEYS = ("requested_vx_mps", "requested_vy_mps", "requested_yaw_rad_s")
GOVERNED_KEYS = ("governed_vx_mps", "governed_vy_mps", "governed_yaw_rad_s")


def parse_levels(value):
    """Require finite, increasing positive magnitudes; each is tested both ways."""
    try:
        result = [float(item.strip()) for item in value.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("levels must be comma-separated numbers") from error
    if (not result or not all(math.isfinite(item) and item > 0 for item in result)
            or any(right <= left for left, right in zip(result, result[1:]))):
        raise argparse.ArgumentTypeError("levels must be finite positive and strictly increasing")
    return result


def body_velocity(q, velocity):
    """Inverse quaternion rotation of the adapter's world-frame linear twist."""
    norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    w, x, y, z = (value / norm for value in (q.w, q.x, q.y, q.z))
    vx, vy, vz = velocity
    return (
        (1-2*(y*y+z*z))*vx + 2*(x*y+z*w)*vy + 2*(x*z-y*w)*vz,
        2*(x*y-z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z+x*w)*vz,
        2*(x*z+y*w)*vx + 2*(y*z-x*w)*vy + (1-2*(x*x+y*y))*vz,
    )


def weighted_mean(samples, key, squared_error_target=None):
    """Integrate distinct truth timestamps rather than wall callback frequency."""
    integral = duration = 0.0
    for left, right in zip(samples, samples[1:]):
        dt = right["stamp"] - left["stamp"]
        if dt <= 0:
            continue
        values = (left[key], right[key])
        if squared_error_target is not None:
            values = tuple((value-squared_error_target)**2 for value in values)
        integral += 0.5 * sum(values) * dt
        duration += dt
    if duration <= 0:
        raise RuntimeError("insufficient distinct truth timestamps for speed measurement")
    mean = integral / duration
    return math.sqrt(mean) if squared_error_target is not None else mean


class SpeedSweepAcceptance(PlanarAcceptance):
    def __init__(self, args):
        self.phase_samples = None
        self.current_result = None
        self.previous_truth_stamp = None
        self.previous_truth_yaw = None
        self.unwrapped_yaw = 0.0
        super().__init__(args)

    def publish_legacy_joy(self, vx, vy, yaw, button):
        # Explicit normalization matches the isolated simulation controller
        # profile. Requested-command diagnostics independently catch a mismatch.
        normalized = (-yaw/self.args.joy_limit_yaw,
                      -vx/self.args.joy_limit_x, -vy/self.args.joy_limit_y)
        if any(abs(value) > 1.000001 for value in normalized):
            raise RuntimeError("command exceeds explicitly configured legacy Joy limits")
        joy = Joy()
        joy.axes = [normalized[0], normalized[1], -1.0, normalized[2], 0.0, -1.0]
        joy.buttons = [0]*10
        if button is not None:
            joy.buttons[button] = 1
            if button != 7:
                joy.axes[2] = 1.0
        self.motion_guard.set_command(vx, vy)
        self.joy_publisher.publish(joy)

    def on_truth(self, message):
        if message.header.frame_id != "world":
            raise RuntimeError("speed sweep requires world-frame truth twist from Fortress adapter")
        super().on_truth(message)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if self.previous_truth_stamp is not None and stamp <= self.previous_truth_stamp:
            return
        yaw_rate = 0.0
        if self.previous_truth_stamp is not None:
            delta = angle_delta(self.truth["yaw"], self.previous_truth_yaw)
            self.unwrapped_yaw += delta
            yaw_rate = delta/(stamp-self.previous_truth_stamp)
        self.previous_truth_stamp, self.previous_truth_yaw = stamp, self.truth["yaw"]
        if self.phase_samples is not None:
            body = body_velocity(message.pose.pose.orientation,
                                 tuple(self.truth[key] for key in ("vx", "vy", "vz")))
            self.phase_samples.append(dict(self.truth, stamp=stamp,
                body_vx=body[0], body_vy=body[1], body_vz=body[2],
                yaw_rate=yaw_rate, unwrapped_yaw=self.unwrapped_yaw))

    def startup(self):
        if self.args.already_standing:
            self.wait_standing_ready()
            return
        self.wait_ready()
        if self.args.legacy_joy:
            start = self.seconds()
            while self.seconds()-start < 0.6:
                self.publish(0.0, button=7)
                rclpy.spin_once(self, timeout_sec=0.03)
                self.check_telemetry()
                if self.mode not in ("PASSIVE", "CALIBRATION"):
                    raise RuntimeError("legacy START must not request standing")
            if self.latest.get("requires_recalibration") != "false":
                raise RuntimeError("legacy START did not finish calibration gate")
            self.wait_mode("MPC_STANCE", 20.0, button=0)
        else:
            self.wait_mode("MPC_STANCE", 20.0, button=7)
        if not any(item["mode"] == "STAND_UP" for item in self.mode_history):
            raise RuntimeError("standing request skipped interpolated STAND_UP")

    def finish_phase_measurements(self, result, start_pose, start_yaw):
        samples = self.phase_samples or []
        if not samples:
            return
        result["truth_samples"] = len(samples)
        result["final_truth"] = dict(self.truth)
        result["elapsed_sim_s"] = self.seconds()-result["start_sim_s"]
        result["yaw_displacement_rad"] = self.unwrapped_yaw-start_yaw
        dx, dy = self.truth["x"]-start_pose["x"], self.truth["y"]-start_pose["y"]
        result["forward_displacement_m"] = math.cos(start_pose["yaw"])*dx + math.sin(start_pose["yaw"])*dy
        result["lateral_displacement_m"] = -math.sin(start_pose["yaw"])*dx + math.cos(start_pose["yaw"])*dy
        for axis in ("roll", "pitch"):
            result["max_abs_"+axis+"_rad"] = max(abs(item[axis]) for item in samples)
        result["min_height_m"] = min(item["z"] for item in samples)
        result["max_height_m"] = max(item["z"] for item in samples)
        cutoff = samples[-1]["stamp"]-self.args.measurement_window
        steady = [item for item in samples if item["stamp"] >= cutoff
                  and item["stamp"] >= result["start_sim_s"]+self.args.command_settle_duration]
        if len(steady) < 2:
            return
        command = result["command"]
        result["steady_truth_duration_s"] = steady[-1]["stamp"]-steady[0]["stamp"]
        result["steady_mean_body_vx_mps"] = weighted_mean(steady, "body_vx")
        result["steady_mean_body_vy_mps"] = weighted_mean(steady, "body_vy")
        result["steady_mean_yaw_rate_rad_s"] = weighted_mean(steady, "yaw_rate")
        for key, target, suffix in (("body_vx", command[0], "vx_mps"),
                                    ("body_vy", command[1], "vy_mps"),
                                    ("yaw_rate", command[2], "yaw_rad_s")):
            result["steady_"+suffix+"_error"] = weighted_mean(steady, key)-target
            result["steady_"+suffix+"_rmse"] = weighted_mean(steady, key, target)

    def sweep_phase(self, axis, magnitude, sign):
        index = {"x": 0, "y": 1, "yaw": 2}[axis]
        command = [0.0, 0.0, 0.0]
        command[index] = sign*magnitude
        name = f"{axis}_{'positive' if sign > 0 else 'negative'}_{magnitude:g}"
        result = {"name": name, "command": command, "tested_axis": axis,
                  "passed": False, "stability_passed": False, "tracking_passed": False,
                  "tracking_failures": [], "duration_sim_s": self.args.phase_duration,
                  "diagnostic_samples": 0, "steady_requested_samples": 0,
                  "invalid_wbc_samples": 0, "policy_modes": [],
                  "max_requested_command_error": [0.0, 0.0, 0.0],
                  "start_sim_s": self.seconds()}
        self.phases.append(result)
        self.current_result = result
        self.motion_guard.begin_phase(result)
        self.phase_samples = []
        diagnostic_samples = []
        start_pose, start_yaw = dict(self.truth), self.unwrapped_yaw
        start_wall = time.monotonic()
        try:
            # Include Trot-entry failures in this stage's retained record.
            self.wait_trot(command)
            result["start_sim_s"] = self.seconds()
            self.phase_samples = []
            start_pose, start_yaw = dict(self.truth), self.unwrapped_yaw
            seen_sequence, modes, next_print = -1, set(), start_wall
            while rclpy.ok() and self.seconds()-result["start_sim_s"] < self.args.phase_duration:
                self.publish(command[0], vy=command[1], yaw=command[2])
                rclpy.spin_once(self, timeout_sec=0.03)
                self.check_telemetry(active=True)
                elapsed = self.seconds()-result["start_sim_s"]
                if self.mode != "MPC_TROT":
                    raise RuntimeError(name+": unexpected mode "+self.mode)
                if elapsed < -0.01:
                    raise RuntimeError("simulation clock moved backwards")
                if time.monotonic()-start_wall > max(30.0, (self.args.phase_duration+6)*self.args.wall_factor):
                    raise RuntimeError(name+": physics stalled")
                if self.diag_sequence != seen_sequence:
                    seen_sequence = self.diag_sequence
                    result["diagnostic_samples"] += 1
                    modes.add(int(self.latest.get("policy_mode", "-1")))
                    result["policy_modes"] = sorted(modes)
                    if elapsed > 0.4:
                        result["invalid_wbc_samples"] += self.latest.get("wbc_valid") != "true"
                    if elapsed >= self.args.command_settle_duration:
                        requested = [float(self.latest.get(key, "nan")) for key in REQUEST_KEYS]
                        governed = [float(self.latest.get(key, "nan")) for key in GOVERNED_KEYS]
                        if not all(math.isfinite(value) for value in requested+governed):
                            raise RuntimeError(name+": requested/governed velocity diagnostics missing/non-finite")
                        diagnostic_samples.append({"stamp": self.seconds(),
                            **dict(zip(REQUEST_KEYS, requested)), **dict(zip(GOVERNED_KEYS, governed))})
                        result["steady_requested_samples"] += 1
                        for idx, (actual, target) in enumerate(zip(requested, command)):
                            error = abs(actual-target)
                            result["max_requested_command_error"][idx] = max(error, result["max_requested_command_error"][idx])
                            tolerance = self.args.requested_yaw_tolerance if idx == 2 else self.args.requested_speed_tolerance
                            if error > tolerance:
                                raise RuntimeError(f"{name}: controller requested {requested}, commanded {command}; "
                                                   "speed limits, Joy normalization or slew prevented the intended test")
                if time.monotonic() >= next_print:
                    print(f"{name}: sim={elapsed:.1f}s z={self.truth['z']:.3f} "
                          f"xy=({self.truth['x']:.3f},{self.truth['y']:.3f})", flush=True)
                    next_print = time.monotonic()+2
            self.finish_phase_measurements(result, start_pose, start_yaw)
            if result["diagnostic_samples"] < 3 or result["steady_requested_samples"] < 3:
                raise RuntimeError(name+": insufficient fresh requested-command diagnostics")
            if result["invalid_wbc_samples"] or not {6, 9}.issubset(modes):
                raise RuntimeError(name+": invalid WBC or missing diagonal Trot contact modes")
            if result.get("steady_truth_duration_s", 0) < self.args.measurement_window*0.8:
                raise RuntimeError(name+": insufficient steady independent truth measurements")
            error_key = ("steady_vx_mps_error", "steady_vy_mps_error", "steady_yaw_rad_s_error")[index]
            absolute = self.args.yaw_error_absolute if index == 2 else self.args.speed_error_absolute
            tolerance = max(absolute, self.args.speed_error_relative*magnitude)
            result["steady_speed_error_limit"] = tolerance
            if abs(result[error_key]) > tolerance:
                result["tracking_failures"].append(f"steady velocity error {result[error_key]:.3f} exceeds {tolerance:.3f}")
            displacement = result[("forward_displacement_m", "lateral_displacement_m", "yaw_displacement_rad")[index]]
            if sign*displacement < magnitude*self.args.phase_duration*0.25:
                result["tracking_failures"].append("insufficient displacement in commanded direction")
            if axis == "yaw":
                if math.hypot(result["forward_displacement_m"], result["lateral_displacement_m"]) > self.args.max_turn_translation:
                    result["tracking_failures"].append("excessive translation during in-place yaw")
            else:
                off_axis = result["lateral_displacement_m" if axis == "x" else "forward_displacement_m"]
                if abs(off_axis) > self.args.max_off_axis or abs(result["yaw_displacement_rad"]) > self.args.max_yaw_drift:
                    result["tracking_failures"].append("excessive off-axis/yaw drift")
            result["stability_passed"] = True
            print("PHASE_STABILITY_PASS "+name, flush=True)
        except (RuntimeError, ValueError) as error:
            result["error"] = str(error)
            raise
        finally:
            self.finish_phase_measurements(result, start_pose, start_yaw)
            if diagnostic_samples:
                cutoff = diagnostic_samples[-1]["stamp"]-self.args.measurement_window
                steady_diagnostics = [item for item in diagnostic_samples if item["stamp"] >= cutoff]
                result["steady_diagnostic_command_stats"] = {
                    key: {"mean": sum(item[key] for item in steady_diagnostics)/len(steady_diagnostics),
                          "min": min(item[key] for item in steady_diagnostics),
                          "max": max(item[key] for item in steady_diagnostics)}
                    for key in REQUEST_KEYS+GOVERNED_KEYS}
                governed_error = abs(result["steady_diagnostic_command_stats"][GOVERNED_KEYS[index]]["mean"]-command[index])
                result["steady_governed_command_error"] = governed_error
                governed_tolerance = self.args.requested_yaw_tolerance if index == 2 else self.args.requested_speed_tolerance
                if governed_error > governed_tolerance:
                    result["tracking_failures"].append(
                        f"governed reference differs from command by {governed_error:.3f}; reference lead limiter remains enabled")
            result["tracking_passed"] = result["stability_passed"] and not result["tracking_failures"]
            result["passed"] = result["stability_passed"] and result["tracking_passed"]
            if result["stability_passed"] and not result["tracking_passed"]:
                print("PHASE_TRACKING_FAIL "+name+": "+"; ".join(result["tracking_failures"]), flush=True)
            result["elapsed_wall_s"] = time.monotonic()-start_wall
            self.phase_samples = self.current_result = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--axis", choices=("all", "x", "y", "yaw"), default="all")
    parser.add_argument("--direction", choices=("both", "positive", "negative"), default="both")
    parser.add_argument("--vx-levels", type=parse_levels, default=parse_levels("0.1,0.2,0.3,0.4,0.5"))
    parser.add_argument("--vy-levels", type=parse_levels, default=parse_levels("0.1,0.2,0.3"))
    parser.add_argument("--yaw-levels", type=parse_levels, default=parse_levels("0.4,0.8,1.2"))
    parser.add_argument("--already-standing", action="store_true")
    parser.add_argument("--legacy-joy", action="store_true")
    parser.add_argument("--joy-limit-x", type=float, default=0.5)
    parser.add_argument("--joy-limit-y", type=float, default=0.5)
    parser.add_argument("--joy-limit-yaw", type=float, default=1.5)
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--startup-timeout", type=float, default=900.0)
    parser.add_argument("--wall-factor", type=float, default=25.0)
    parser.add_argument("--telemetry-timeout", type=float, default=5.0)
    parser.add_argument("--phase-duration", type=float, default=6.0)
    parser.add_argument("--settle-duration", type=float, default=2.0)
    parser.add_argument("--soak-duration", type=float, default=30.0)
    parser.add_argument("--command-settle-duration", type=float, default=3.0)
    parser.add_argument("--measurement-window", type=float, default=2.0)
    parser.add_argument("--requested-speed-tolerance", type=float, default=0.005)
    parser.add_argument("--requested-yaw-tolerance", type=float, default=0.01)
    parser.add_argument("--speed-error-absolute", type=float, default=0.10)
    parser.add_argument("--yaw-error-absolute", type=float, default=0.20)
    parser.add_argument("--speed-error-relative", type=float, default=0.40)
    parser.add_argument("--min-height", type=float, default=0.31)
    parser.add_argument("--max-height", type=float, default=0.36)
    parser.add_argument("--max-tilt", type=float, default=0.25)
    parser.add_argument("--max-off-axis", type=float, default=0.25)
    parser.add_argument("--max-yaw-drift", type=float, default=0.35)
    parser.add_argument("--max-turn-translation", type=float, default=0.25)
    add_motion_safety_arguments(parser)
    args = parser.parse_args()
    positive = [value for name, value in vars(args).items()
                if isinstance(value, float) and name != "soak_duration"]
    if not all(math.isfinite(value) and value > 0 for value in positive):
        parser.error("all timeouts and limits must be finite positive numbers")
    if (args.min_height >= args.max_height or not 4 <= args.phase_duration <= 30
            or args.command_settle_duration+args.measurement_window > args.phase_duration
            or not math.isfinite(args.soak_duration) or not 0 <= args.soak_duration <= 600):
        parser.error("require min-height < max-height, phase 4..30 s, settle+measurement <= phase, soak 0..600 s")
    levels = {"x": args.vx_levels, "y": args.vy_levels, "yaw": args.yaw_levels}
    axes = list(levels) if args.axis == "all" else [args.axis]
    if any(max(levels[axis]) >= args.max_horizontal_speed for axis in axes if axis != "yaw"):
        parser.error("set an explicit max-horizontal-speed above every tested translation command")
    rclpy.init(args=[])
    node = SpeedSweepAcceptance(args)
    report = {"passed": False, "stability_passed": False, "velocity_tracking_passed": False,
              "sweep_completed": False, "backend": "Gazebo Fortress physics", "hardware_io": False,
              "simulation_preflight_passed": False,
              "timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "settings": vars(args) | {"output": str(args.output)},
              "scope": "Signed single-axis Trot speed sweep; independent world truth rotated to body frame; faults stop the sweep."}
    code = 1
    try:
        node.startup()
        report["simulation_preflight_passed"] = True
        node.hold("initial_stance", 3.0, "MPC_STANCE")
        signs = {"both": (1, -1), "positive": (1,), "negative": (-1,)}[args.direction]
        for axis in axes:
            for magnitude in levels[axis]:
                for sign in signs:
                    node.sweep_phase(axis, magnitude, sign)
                    node.settle(f"zero_speed_stance_after_{axis}_{sign*magnitude:g}")
        node.wait_mode("MPC_STANCE", 8.0, button=0)
        node.hold("final_A_zero_speed_stance", args.settle_duration, "MPC_STANCE", require_disarmed=True)
        if args.soak_duration:
            node.hold("final_stance_soak", args.soak_duration, "MPC_STANCE", require_disarmed=True)
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown interrupted the speed sweep")
        report["sweep_completed"] = report["stability_passed"] = True
        motion_phases = [phase for phase in node.phases if "tracking_passed" in phase]
        report["velocity_tracking_passed"] = all(phase["tracking_passed"] for phase in motion_phases)
        report["passed"] = report["stability_passed"] and report["velocity_tracking_passed"]
        code = 0 if report["passed"] else 1
        print(("PASS" if report["passed"] else "TRACKING_FAIL")+
              ": signed Trot speed sweep completed with final stance", flush=True)
    except (RuntimeError, ValueError) as error:
        report["error"] = str(error)
        print("FAIL: "+str(error), file=sys.stderr, flush=True)
    finally:
        report.update(phases=node.phases, mode_history=node.mode_history,
                      final_mode=node.mode, final_diagnostics=node.latest,
                      final_status_message=node.status_message,
                      motion_safety=node.motion_guard.summary)
        if node.simulation_verified:
            try:
                for _ in range(5):
                    node.publish(0.0, button=0)
                    rclpy.spin_once(node, timeout_sec=0.02)
            except (RuntimeError, rclpy.exceptions.RCLError):
                pass
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
