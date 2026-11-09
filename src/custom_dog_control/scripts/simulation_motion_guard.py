"""Independent physics velocity guards for low-speed simulation acceptance."""

import math


def add_motion_safety_arguments(parser):
    parser.add_argument("--max-horizontal-speed", type=float, default=0.35,
                        help="physics body-speed limit for this low-speed acceptance (m/s)")
    parser.add_argument("--overspeed-duration", type=float, default=0.25,
                        help="continuous physics overspeed dwell before failure (simulation s)")
    parser.add_argument("--reverse-speed-threshold", type=float, default=0.05,
                        help="minimum wrong-way speed magnitude (m/s)")
    parser.add_argument("--reverse-duration", type=float, default=1.0,
                        help="continuous wrong-way dwell before failure (simulation s)")


class PhysicsMotionGuard:
    """Early failure checks using independent world-frame physics velocity.

    These limits belong to the low-speed simulation acceptance, not the
    production controller. Dwell uses odometry stamps, so queued callbacks or
    a paused simulator cannot manufacture elapsed time.
    """
    def __init__(self, args):
        self.max_speed = args.max_horizontal_speed
        self.overspeed_duration = args.overspeed_duration
        self.reverse_threshold = args.reverse_speed_threshold
        self.reverse_duration = args.reverse_duration
        self.command = (0.0, 0.0)
        self.last_stamp = None
        self.overspeed_since = self.reverse_since = None
        self.phase = None
        self.summary = {
            "velocity_source": args.truth_topic,
            "velocity_frame": "world (Fortress odometry adapter contract)",
            "max_horizontal_speed_limit_mps": self.max_speed,
            "overspeed_failure_duration_sim_s": self.overspeed_duration,
            "reverse_speed_threshold_mps": self.reverse_threshold,
            "reverse_failure_duration_sim_s": self.reverse_duration,
            "max_horizontal_speed_mps": 0.0,
            "min_command_projected_speed_mps": None,
            "max_overspeed_duration_sim_s": 0.0,
            "max_reverse_duration_sim_s": 0.0,
            "failure": None,
        }

    def set_command(self, vx, vy):
        command = (float(vx), float(vy))
        if command != self.command:
            self.reverse_since = None
        self.command = command

    def begin_phase(self, result):
        result["motion_safety"] = {
            "max_horizontal_speed_mps": 0.0,
            "min_command_projected_speed_mps": None,
            "max_overspeed_duration_sim_s": 0.0,
            "max_reverse_duration_sim_s": 0.0,
        }
        self.phase = result["motion_safety"]

    def observe(self, stamp, truth, mode, armed):
        if self.last_stamp is not None and stamp < self.last_stamp - 0.01:
            raise RuntimeError("independent physics odometry time moved backwards")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if not armed or mode not in ("MPC_STANCE", "MPC_TROT"):
            self.overspeed_since = self.reverse_since = None
            return
        speed = math.hypot(truth["vx"], truth["vy"])
        command_norm = math.hypot(*self.command)
        projected = None
        if command_norm > 0.01 and mode == "MPC_TROT":
            # /cmd_vel is body-frame. Rotate its direction with current true
            # yaw, including lateral motion and turns; never compare world x.
            vx, vy = self.command
            yaw = truth["yaw"]
            world_x = math.cos(yaw) * vx - math.sin(yaw) * vy
            world_y = math.sin(yaw) * vx + math.cos(yaw) * vy
            projected = (truth["vx"] * world_x + truth["vy"] * world_y) / command_norm
        overspeed = speed > self.max_speed
        reverse = projected is not None and projected < -max(self.reverse_threshold, 0.5 * command_norm)
        self.overspeed_since = (stamp if self.overspeed_since is None else self.overspeed_since) if overspeed else None
        self.reverse_since = (stamp if self.reverse_since is None else self.reverse_since) if reverse else None
        over_elapsed = 0.0 if self.overspeed_since is None else stamp - self.overspeed_since
        reverse_elapsed = 0.0 if self.reverse_since is None else stamp - self.reverse_since
        for stats in (self.summary, self.phase):
            if stats is None:
                continue
            stats["max_horizontal_speed_mps"] = max(stats["max_horizontal_speed_mps"], speed)
            stats["max_overspeed_duration_sim_s"] = max(stats["max_overspeed_duration_sim_s"], over_elapsed)
            stats["max_reverse_duration_sim_s"] = max(stats["max_reverse_duration_sim_s"], reverse_elapsed)
            if projected is not None:
                previous = stats["min_command_projected_speed_mps"]
                stats["min_command_projected_speed_mps"] = projected if previous is None else min(previous, projected)
        failure = None
        if over_elapsed >= self.overspeed_duration:
            failure = (f"independent physics horizontal speed {speed:.3f} m/s exceeded "
                       f"{self.max_speed:.3f} m/s for {over_elapsed:.3f} simulation s")
        elif reverse_elapsed >= self.reverse_duration:
            failure = (f"independent physics motion opposed the translation command "
                       f"({projected:.3f} m/s projected) for {reverse_elapsed:.3f} simulation s")
        if failure is not None:
            self.summary["failure"] = {
                "message": failure, "truth_stamp_sim_s": stamp, "mode": mode,
                "command_vx_mps": self.command[0], "command_vy_mps": self.command[1],
                "horizontal_speed_mps": speed, "command_projected_speed_mps": projected,
            }
            raise RuntimeError(failure)
