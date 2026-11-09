#!/usr/bin/env python3
"""Adapt Fortress 6.18 odometry to the controller's world-twist contract.

Fortress OdometryPublisher emits body linear velocity and RPY Euler rates.
The existing ground-truth input explicitly consumes world linear/angular
velocity. This internal topic is not used to publish TF.
"""

import copy
import math


def world_twist(orientation_xyzw, linear_body_xyz, euler_rates_rpy):
    values = (*orientation_xyzw, *linear_body_xyz, *euler_rates_rpy)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite odometry")
    norm = math.sqrt(sum(value * value for value in orientation_xyzw))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid orientation")
    x, y, z, w = (value / norm for value in orientation_xyzw)
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    linear_world = tuple(sum(row[i] * linear_body_xyz[i] for i in range(3))
                         for row in rotation)
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    roll_dot, pitch_dot, yaw_dot = euler_rates_rpy
    angular_world = (
        roll_dot * math.cos(yaw) * math.cos(pitch) - pitch_dot * math.sin(yaw),
        roll_dot * math.sin(yaw) * math.cos(pitch) + pitch_dot * math.cos(yaw),
        yaw_dot - roll_dot * math.sin(pitch),
    )
    return linear_world, angular_world


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class Adapter(Node):
        def __init__(self):
            super().__init__("fortress_odom_adapter")
            self.declare_parameter("input_topic", "/fortress/odom")
            self.declare_parameter("output_topic", "/ground_truth/odom")
            self.first_stamp = None
            self.publisher = self.create_publisher(
                Odometry, self.get_parameter("output_topic").value,
                qos_profile_sensor_data)
            self.subscription = self.create_subscription(
                Odometry, self.get_parameter("input_topic").value,
                self.convert, qos_profile_sensor_data)

        def convert(self, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if self.first_stamp is None or stamp < self.first_stamp:
                self.first_stamp = stamp
            # Fortress initializes its differencing pose on startup. Wait for
            # its 10-sample velocity window to discard that first transient.
            if stamp - self.first_stamp < 0.05:
                return
            q = message.pose.pose.orientation
            v = message.twist.twist.linear
            rates = message.twist.twist.angular
            try:
                linear, angular = world_twist(
                    (q.x, q.y, q.z, q.w), (v.x, v.y, v.z),
                    (rates.x, rates.y, rates.z))
            except ValueError:
                return
            output = copy.deepcopy(message)
            output.header.frame_id = "world"
            output.child_frame_id = "base"
            output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = linear
            output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = angular
            self.publisher.publish(output)

    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
