#!/usr/bin/env python3
"""Normalize the Livox MID-360 IMU message for robot_localization.

The Livox ROS driver publishes sensor_msgs/Imu on /livox/imu, labels the
message livox_frame, and forwards the protocol acceleration values directly.
The MID-360 protocol reports acceleration in g, while sensor_msgs/Imu uses
m/s^2. This adapter fixes the frame label and acceleration units and supplies
non-zero covariances. The current EKF intentionally consumes only angular
velocity.z from the result.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _diagonal_covariance(variance: float):
    covariance = [0.0] * 9
    for index in range(3):
        covariance[index * 3 + index] = variance
    return covariance


class LivoxImuAdapter(Node):
    def __init__(self) -> None:
        super().__init__("livox_imu_adapter")
        self.input_topic = self.declare_parameter(
            "input_topic", "/livox/imu"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/target/imu/livox"
        ).value
        self.output_frame = self.declare_parameter(
            "output_frame", "target/livox_mount_link"
        ).value
        self.acceleration_scale = float(self.declare_parameter(
            "acceleration_scale", 9.80665
        ).value)
        self.gyro_variance = float(self.declare_parameter(
            "gyro_variance", 0.0004
        ).value)
        self.acceleration_variance = float(self.declare_parameter(
            "acceleration_variance", 0.25
        ).value)

        self.publisher = self.create_publisher(Imu, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self._callback, 20
        )
        self.get_logger().info(
            f"Converting {self.input_topic} to {self.output_topic} in frame "
            f"{self.output_frame} (acceleration scale "
            f"{self.acceleration_scale:.5f})"
        )

    def _callback(self, message: Imu) -> None:
        output = Imu()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame

        output.angular_velocity = message.angular_velocity
        output.linear_acceleration.x = (
            message.linear_acceleration.x * self.acceleration_scale
        )
        output.linear_acceleration.y = (
            message.linear_acceleration.y * self.acceleration_scale
        )
        output.linear_acceleration.z = (
            message.linear_acceleration.z * self.acceleration_scale
        )

        output.angular_velocity_covariance = _diagonal_covariance(
            max(self.gyro_variance, 1.0e-9)
        )
        output.linear_acceleration_covariance = _diagonal_covariance(
            max(self.acceleration_variance, 1.0e-9)
        )

        # The Livox driver does not publish an attitude estimate. Mark it as
        # unavailable instead of passing a zero quaternion with zero covariance.
        output.orientation_covariance[0] = -1.0
        output.orientation.x = 0.0
        output.orientation.y = 0.0
        output.orientation.z = 0.0
        output.orientation.w = 1.0
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
