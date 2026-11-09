#!/usr/bin/env python3
"""Read-only Fortress pose/joint recorder with independent four-foot FK.

No control commands are published and no hardware ports are opened. Clearance
is URDF sphere-center world height minus radius and the configured flat ground.
"""

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import time


LEGS = ('FR', 'FL', 'RR', 'RL')
JOINTS = tuple(f'{leg}_{part}_joint' for leg in LEGS
               for part in ('hip', 'thigh', 'calf'))
POSE_FIELDS = ('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad')


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_values(message):
    pose = message.pose.pose
    position = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(value) for value in (*position, *quaternion)):
        raise ValueError('non-finite pose')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = (value / norm for value in quaternion)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1., min(1., 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return position, (x, y, z, w), (*position, roll, pitch, yaw)


def finite_range(values):
    values = [float(value) for value in values if math.isfinite(value)]
    return {'min': min(values), 'max': max(values)} if values else None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new CSV file')
    parser.add_argument('--duration', type=float, default=60., help='simulation seconds')
    parser.add_argument('--urdf', type=Path, help='defaults to custom_dog_description canonical URDF')
    parser.add_argument('--sample-hz', type=float, default=20.)
    parser.add_argument('--max-skew', type=float, default=.03, help='truth/joint timestamp tolerance, seconds')
    parser.add_argument('--ground-z', type=float, default=0.)
    parser.add_argument('--foot-radius', type=float, default=.026)
    parser.add_argument('--wall-timeout', type=float, default=1800.)
    args = parser.parse_args()
    if not all(math.isfinite(x) and x > 0 for x in
               (args.duration, args.sample_hz, args.max_skew, args.foot_radius,
                args.wall_timeout)) or not math.isfinite(args.ground_z):
        parser.error('timing/radius must be positive finite values; ground height must be finite')
    if args.output.exists():
        parser.error('output exists; choose a new CSV filename to retain previous evidence')
    return args


def main():
    args = parse_args()
    # Keep --help independent of the ROS and Pinocchio runtime environment.
    import numpy as np
    import pinocchio as pin
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    if args.urdf is None:
        args.urdf = Path(get_package_share_directory('custom_dog_description')) / 'urdf/custom_dog.urdf'
    args.urdf = args.urdf.resolve(strict=True)
    model = pin.buildModelFromUrdf(str(args.urdf), pin.JointModelFreeFlyer())
    data = model.createData()
    if model.nq != 19 or any(name not in model.names for name in JOINTS):
        raise RuntimeError('expected a free base and the 12 named single-axis joints')
    q_indices = [model.joints[model.getJointId(name)].idx_q for name in JOINTS]
    foot_names = tuple(f'{leg}_foot' for leg in LEGS)
    if any(name not in {frame.name for frame in model.frames} for name in foot_names):
        raise RuntimeError('canonical model is missing a foot frame')
    foot_ids = [model.getFrameId(name) for name in foot_names]

    fields = ['sim_time_s', 'elapsed_sim_s', 'mode', 'policy_mode',
              'truth_stamp_s', 'joint_stamp_s', 'truth_joint_skew_s',
              'diagnostics_stamp_s', 'mode_age_s', 'estimate_stamp_s',
              'estimate_valid', 'max_abs_joint_effort_nm', 'diag_wbc_ff_max_nm']
    fields += [f'truth_{field}' for field in POSE_FIELDS]
    fields += [f'estimate_{field}' for field in POSE_FIELDS]
    fields += [f'{leg}_{field}' for leg in LEGS
               for field in ('world_x_m', 'world_y_m', 'world_z_m',
                             'clearance_m', 'planned_contact')]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_file = args.output.open('x', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    class Recorder(Node):
        def __init__(self):
            super().__init__('fortress_kinematics_recorder',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.truth = self.joints = self.estimate = None
            self.diagnostics = None
            self.rows = []
            self.rejected = Counter()
            self.start = None
            self.done = False
            self.reason = 'running'
            self.simulation_verified = False
            self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(JointState, '/joint_states', self.on_joints,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_estimate,
                                     qos_profile_sensor_data)
            self.create_subscription(DiagnosticArray, '/nmpc_wbc_controller/diagnostics',
                                     self.on_diagnostics, 10)
            self.create_timer(1. / args.sample_hz, self.sample)

        def on_truth(self, message):
            try:
                position, quaternion, pose = pose_values(message)
            except ValueError:
                self.rejected['invalid_truth_pose'] += 1
                return
            stamp = stamp_seconds(message.header.stamp)
            if stamp > 0:
                self.truth = (stamp, position, quaternion, pose)

        def on_estimate(self, message):
            try:
                _, _, pose = pose_values(message)
            except ValueError:
                self.rejected['invalid_estimate_pose'] += 1
                return
            self.estimate = (stamp_seconds(message.header.stamp), pose)

        def on_joints(self, message):
            positions = dict(zip(message.name, message.position))
            if any(name not in positions or not math.isfinite(positions[name]) for name in JOINTS):
                self.rejected['incomplete_or_invalid_joints'] += 1
                return
            efforts = dict(zip(message.name, message.effort))
            maximum = max(abs(efforts[name]) for name in JOINTS) if all(
                name in efforts and math.isfinite(efforts[name]) for name in JOINTS) else math.nan
            self.joints = (stamp_seconds(message.header.stamp), positions, maximum)

        def on_diagnostics(self, message):
            for status in message.status:
                if status.name != 'custom_dog_control/nmpc_wbc':
                    continue
                if status.hardware_id != 'gazebo':
                    self.reason = 'non_simulation_diagnostics'
                    self.done = True
                    return
                self.simulation_verified = True
                values = {item.key: item.value for item in status.values}
                self.diagnostics = (stamp_seconds(message.header.stamp), values)

        def sample(self):
            if self.done:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.start is not None and now - self.start >= args.duration:
                self.done, self.reason = True, 'duration_complete'
                return
            if self.truth is None or self.joints is None or self.diagnostics is None:
                self.rejected['waiting_for_inputs'] += 1
                return
            truth_stamp, position, quaternion, pose = self.truth
            joint_stamp, positions, effort = self.joints
            diag_stamp, diagnostics = self.diagnostics
            skew = abs(truth_stamp - joint_stamp)
            if skew > args.max_skew or max(abs(now - truth_stamp), abs(now - joint_stamp)) > args.max_skew:
                self.rejected['truth_joint_timestamp_skew'] += 1
                return
            # Mode is published at 10 Hz; holding its last label for <=0.2 s
            # preserves 20 Hz FK sampling. Its exact age is retained in CSV.
            if not -args.max_skew <= truth_stamp - diag_stamp <= .2:
                self.rejected['stale_or_future_mode'] += 1
                return
            if self.rows and truth_stamp <= self.rows[-1]['sim_time_s']:
                self.rejected['duplicate_or_backward_truth_time'] += 1
                return
            q = pin.neutral(model)
            q[:3], q[3:7] = position, quaternion
            for index, name in zip(q_indices, JOINTS):
                q[index] = positions[name]
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            if self.start is None:
                self.start = truth_stamp
            try:
                policy_mode = int(diagnostics.get('policy_mode', '-1'))
                ff_max = float(diagnostics.get('wbc_tau_max_abs', 'nan'))
            except ValueError:
                policy_mode, ff_max = -1, math.nan
            row = dict.fromkeys(fields, math.nan)
            row.update(sim_time_s=truth_stamp, elapsed_sim_s=truth_stamp - self.start,
                       mode=diagnostics.get('mode', 'UNKNOWN'), policy_mode=policy_mode,
                       truth_stamp_s=truth_stamp, joint_stamp_s=joint_stamp,
                       truth_joint_skew_s=skew, diagnostics_stamp_s=diag_stamp,
                       mode_age_s=truth_stamp - diag_stamp, estimate_valid=0,
                       max_abs_joint_effort_nm=effort, diag_wbc_ff_max_nm=ff_max)
            row.update(zip((f'truth_{field}' for field in POSE_FIELDS), pose))
            if self.estimate is not None:
                estimate_stamp, estimate = self.estimate
                row['estimate_stamp_s'] = estimate_stamp
                if abs(estimate_stamp - truth_stamp) <= args.max_skew:
                    row['estimate_valid'] = 1
                    row.update(zip((f'estimate_{field}' for field in POSE_FIELDS), estimate))
            for leg, frame in zip(LEGS, foot_ids):
                foot = data.oMf[frame].translation
                row.update({f'{leg}_world_x_m': float(foot[0]),
                            f'{leg}_world_y_m': float(foot[1]),
                            f'{leg}_world_z_m': float(foot[2]),
                            f'{leg}_clearance_m': float(foot[2]) - args.foot_radius - args.ground_z,
                            f'{leg}_planned_contact': int(bool(policy_mode & (8 >> LEGS.index(leg))))
                            if policy_mode >= 0 else -1})
            writer.writerow(row)
            self.rows.append(row)
            if len(self.rows) % max(1, round(args.sample_hz)) == 0:
                csv_file.flush()

    rclpy.init()
    node = Recorder()
    wall_start = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            if time.monotonic() - wall_start > args.wall_timeout:
                node.reason = 'wall_timeout'
                break
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        node.reason = 'interrupted'
    finally:
        csv_file.flush()
        csv_file.close()
        rows = node.rows
        trot = [row for row in rows if row['mode'] == 'MPC_TROT']
        summary = {
            'hardware_io': False, 'control_commands_published': False,
            'simulation_source_verified': node.simulation_verified,
            'completion_reason': node.reason, 'csv': str(args.output.resolve()),
            'urdf': str(args.urdf), 'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
            'model_mass_kg': float(pin.computeTotalMass(model)),
            'sample_hz_requested': args.sample_hz, 'duration_sim_s_requested': args.duration,
            'truth_joint_max_skew_s_configured': args.max_skew,
            'recorded_samples': len(rows), 'trot_samples': len(trot),
            'foot_frames': list(foot_names), 'foot_radius_m': args.foot_radius,
            'ground_assumption': f'horizontal plane at world z={args.ground_z} m',
            'maximum_truth_joint_skew_s': max((r['truth_joint_skew_s'] for r in rows), default=None),
            'rejected_samples_or_messages': dict(node.rejected),
            'mode_samples': dict(Counter(row['mode'] for row in rows)),
            'body_height_m': finite_range(row['truth_z_m'] for row in rows),
            'trot_body_height_m': finite_range(row['truth_z_m'] for row in trot),
            'trot_max_abs_roll_pitch_rad': max((max(abs(r['truth_roll_rad']), abs(r['truth_pitch_rad']))
                                                 for r in trot), default=None),
            'max_abs_joint_effort_nm': max((r['max_abs_joint_effort_nm'] for r in rows
                                             if math.isfinite(r['max_abs_joint_effort_nm'])), default=None),
            'joint_effort_semantics': 'ROS joint_states effort from simulation; not foot-contact force',
            'trot_foot_clearance_m': {leg: finite_range(row[f'{leg}_clearance_m'] for row in trot)
                                      for leg in LEGS},
        }
        if len(rows) >= 2:
            elapsed = rows[-1]['sim_time_s'] - rows[0]['sim_time_s']
            summary.update(recorded_elapsed_sim_s=elapsed,
                           effective_sample_hz=(len(rows) - 1) / elapsed if elapsed > 0 else None)
        path = args.output.with_suffix('.summary.json')
        path.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        print(json.dumps(summary, indent=2, allow_nan=False))
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.reason == 'duration_complete' and len(rows) > 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
