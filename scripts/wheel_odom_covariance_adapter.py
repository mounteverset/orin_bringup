#!/usr/bin/env python3
"""Apply conservative minimum twist covariances to wheel odometry for fusion.

The Husky base owns the original wheel odometry topic. This node leaves that
topic untouched and republishes a copy for the Orin-side EKF, preventing
optimistic wheel covariances from overpowering visual or inertial corrections
when a skid-steer robot slips.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _minimum_variance(value: float, minimum: float) -> float:
    if not math.isfinite(value) or value < minimum:
        return minimum
    return value


class WheelOdomCovarianceAdapter(Node):
    """Republish wheel odometry with realistic lower-bound twist variances."""

    def __init__(self) -> None:
        super().__init__("wheel_odom_covariance_adapter")

        self.input_topic = str(self.declare_parameter(
            "input_topic", "/target/odometry/filtered"
        ).value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/target/odometry/wheel_fusion"
        ).value)
        self.linear_x_variance = float(self.declare_parameter(
            "linear_x_variance", 0.04
        ).value)
        self.angular_z_variance = float(self.declare_parameter(
            "angular_z_variance", 0.04
        ).value)
        if self.linear_x_variance <= 0.0 or self.angular_z_variance <= 0.0:
            raise ValueError("wheel twist covariance values must be positive")

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._callback, 20
        )
        self.get_logger().info(
            f"Republishing {self.input_topic} as {self.output_topic}; "
            f"minimum variances vx={self.linear_x_variance:.6f}, "
            f"wz={self.angular_z_variance:.6f}"
        )

    def _callback(self, message: Odometry) -> None:
        output = Odometry()
        output.header = message.header
        output.child_frame_id = message.child_frame_id
        output.pose.pose = message.pose.pose
        output.pose.covariance = list(message.pose.covariance)
        output.twist.twist = message.twist.twist
        covariance = list(message.twist.covariance)
        if len(covariance) != 36:
            covariance = [0.0] * 36

        # Twist covariance is row-major [vx, vy, vz, wx, wy, wz].
        covariance[0] = _minimum_variance(
            float(covariance[0]), self.linear_x_variance
        )
        covariance[35] = _minimum_variance(
            float(covariance[35]), self.angular_z_variance
        )
        output.twist.covariance = covariance
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
