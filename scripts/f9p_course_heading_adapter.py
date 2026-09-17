#!/usr/bin/env python3
"""Publish trustworthy F9P course-over-ground samples as an ENU IMU yaw.

A single-antenna F9P does not measure stationary vehicle heading. UBX-NAV-PVT
does provide heading of motion while moving, however. This adapter publishes an
orientation only while the receiver reports a sufficiently accurate course and
the robot odometry confirms straight, forward motion. navsat_transform_node can
then use one of these gated samples to initialize its Cartesian transform.
"""

from collections import deque
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ublox_ubx_msgs.msg import UBXNavPVT


DEGREES_SCALE = 1.0e-5
MILLIMETERS_PER_METER = 1000.0


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def course_heading_to_enu_yaw(head_mot: int) -> float:
    """Convert clockwise degrees from north to CCW radians from east."""
    heading_radians = math.radians(float(head_mot) * DEGREES_SCALE)
    return normalize_angle((math.pi / 2.0) - heading_radians)


def circular_mean(angles) -> float:
    """Return the circular mean of a non-empty sequence of angles."""
    angles = tuple(angles)
    if not angles:
        raise ValueError("circular_mean requires at least one angle")
    return math.atan2(
        sum(math.sin(angle) for angle in angles),
        sum(math.cos(angle) for angle in angles),
    )


class F9PCourseHeadingAdapter(Node):
    def __init__(self) -> None:
        super().__init__("f9p_course_heading_adapter")

        self.input_topic = self.declare_parameter(
            "input_topic", "/gps/ubx_nav_pvt"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/target/odometry/fused"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/target/imu/f9p_heading"
        ).value
        self.output_frame = self.declare_parameter(
            "output_frame", "target/base_link"
        ).value

        self.minimum_ground_speed_mps = float(self.declare_parameter(
            "minimum_ground_speed_mps", 0.30
        ).value)
        self.minimum_forward_speed_mps = float(self.declare_parameter(
            "minimum_forward_speed_mps", 0.15
        ).value)
        self.maximum_abs_yaw_rate_rad_s = float(self.declare_parameter(
            "maximum_abs_yaw_rate_rad_s", 0.08
        ).value)
        self.maximum_heading_accuracy_deg = float(self.declare_parameter(
            "maximum_heading_accuracy_deg", 5.0
        ).value)
        self.maximum_heading_spread_deg = float(self.declare_parameter(
            "maximum_heading_spread_deg", 5.0
        ).value)
        self.maximum_horizontal_accuracy_m = float(self.declare_parameter(
            "maximum_horizontal_accuracy_m", 1.0
        ).value)
        self.minimum_satellites = int(self.declare_parameter(
            "minimum_satellites", 8
        ).value)
        self.required_consecutive_samples = int(self.declare_parameter(
            "required_consecutive_samples", 10
        ).value)
        self.maximum_sample_gap_sec = float(self.declare_parameter(
            "maximum_sample_gap_sec", 0.60
        ).value)
        self.odometry_timeout_sec = float(self.declare_parameter(
            "odometry_timeout_sec", 0.50
        ).value)
        self.require_differential = bool(self.declare_parameter(
            "require_differential", True
        ).value)

        if self.required_consecutive_samples < 1:
            raise ValueError("required_consecutive_samples must be at least 1")
        positive_parameters = {
            "minimum_ground_speed_mps": self.minimum_ground_speed_mps,
            "minimum_forward_speed_mps": self.minimum_forward_speed_mps,
            "maximum_abs_yaw_rate_rad_s": self.maximum_abs_yaw_rate_rad_s,
            "maximum_heading_accuracy_deg": self.maximum_heading_accuracy_deg,
            "maximum_heading_spread_deg": self.maximum_heading_spread_deg,
            "maximum_horizontal_accuracy_m": self.maximum_horizontal_accuracy_m,
            "maximum_sample_gap_sec": self.maximum_sample_gap_sec,
            "odometry_timeout_sec": self.odometry_timeout_sec,
        }
        for name, value in positive_parameters.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

        self._samples = deque(maxlen=self.required_consecutive_samples)
        self._last_pvt_receive_sec = None
        self._last_odom_receive_sec = None
        self._forward_speed_mps = 0.0
        self._yaw_rate_rad_s = 0.0
        self._publishing_valid_segment = False

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.pvt_subscription = self.create_subscription(
            UBXNavPVT, self.input_topic, self._pvt_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 20
        )

        self.get_logger().info(
            f"Waiting for straight forward F9P motion on {self.input_topic}; "
            f"publishing ENU heading to {self.output_topic} after "
            f"{self.required_consecutive_samples} accepted samples"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _odom_callback(self, message: Odometry) -> None:
        self._forward_speed_mps = float(message.twist.twist.linear.x)
        self._yaw_rate_rad_s = float(message.twist.twist.angular.z)
        self._last_odom_receive_sec = self._now_sec()

    def _reject_reason(self, message: UBXNavPVT, now_sec: float):
        if not message.gnss_fix_ok or int(message.gps_fix.fix_type) < 3:
            return "GNSS fix is not a valid 3D fix"
        if message.invalid_llh:
            return "GNSS latitude/longitude is invalid"
        if self.require_differential and not message.diff_soln:
            return "differential corrections are not active"
        if int(message.num_sv) < self.minimum_satellites:
            return "too few satellites"
        if float(message.h_acc) / MILLIMETERS_PER_METER > self.maximum_horizontal_accuracy_m:
            return "horizontal position accuracy is too low"
        if float(message.g_speed) / MILLIMETERS_PER_METER < self.minimum_ground_speed_mps:
            return "GNSS ground speed is too low"
        if float(message.head_acc) * DEGREES_SCALE > self.maximum_heading_accuracy_deg:
            return "GNSS heading accuracy is too low"
        if self._last_odom_receive_sec is None:
            return "odometry has not been received"
        if now_sec - self._last_odom_receive_sec > self.odometry_timeout_sec:
            return "odometry is stale"
        if self._forward_speed_mps < self.minimum_forward_speed_mps:
            return "robot is not moving forward"
        if abs(self._yaw_rate_rad_s) > self.maximum_abs_yaw_rate_rad_s:
            return "robot is turning"
        return None

    def _reset_samples(self, reason: str) -> None:
        self._samples.clear()
        if self._publishing_valid_segment:
            self.get_logger().info(
                f"Stopped F9P heading output: {reason}; waiting for another "
                "straight forward segment"
            )
        self._publishing_valid_segment = False

    def _pvt_callback(self, message: UBXNavPVT) -> None:
        now_sec = self._now_sec()
        if (
            self._last_pvt_receive_sec is not None
            and now_sec - self._last_pvt_receive_sec > self.maximum_sample_gap_sec
        ):
            self._reset_samples("PVT sample gap exceeded the limit")
        self._last_pvt_receive_sec = now_sec

        reason = self._reject_reason(message, now_sec)
        if reason is not None:
            self._reset_samples(reason)
            return

        yaw = course_heading_to_enu_yaw(message.head_mot)
        heading_accuracy_rad = math.radians(
            float(message.head_acc) * DEGREES_SCALE
        )
        self._samples.append((yaw, heading_accuracy_rad))
        if len(self._samples) < self.required_consecutive_samples:
            return

        mean_yaw = circular_mean(sample[0] for sample in self._samples)
        maximum_deviation = max(
            abs(normalize_angle(sample[0] - mean_yaw)) for sample in self._samples
        )
        maximum_spread = math.radians(self.maximum_heading_spread_deg)
        if maximum_deviation > maximum_spread:
            self._reset_samples(
                f"heading spread is {math.degrees(maximum_deviation):.1f} degrees"
            )
            return

        output = Imu()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        output.orientation.z = math.sin(mean_yaw / 2.0)
        output.orientation.w = math.cos(mean_yaw / 2.0)

        yaw_uncertainty = max(
            maximum_deviation,
            max(sample[1] for sample in self._samples),
            math.radians(0.1),
        )
        output.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, yaw_uncertainty * yaw_uncertainty,
        ]
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(output)

        if not self._publishing_valid_segment:
            self.get_logger().info(
                f"Publishing F9P ENU yaw {math.degrees(mean_yaw):.2f} degrees "
                f"with <= {math.degrees(yaw_uncertainty):.2f} degrees reported "
                "uncertainty"
            )
        self._publishing_valid_segment = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = F9PCourseHeadingAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
