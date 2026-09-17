#!/usr/bin/env python3
"""Align private OKVIS odometry to the robot odom frame.

OKVIS publishes the pose of its tracking frame in its own estimator frame. This
node uses one wheel-odometry sample only to establish the initial transform
between the two world frames, then republishes the continuously evolving OKVIS
pose as a pose-only nav_msgs/Odometry measurement in target/odom. It never
publishes TF.
"""

from collections import deque
import math
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


Vector = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w
RigidTransform = Tuple[Vector, Quaternion]


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _normalise_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in q))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in q)  # type: ignore[return-value]


def _quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalise_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def _quaternion_inverse(q: Quaternion) -> Quaternion:
    q = _normalise_quaternion(q)
    return (-q[0], -q[1], -q[2], q[3])


def _rotate(q: Quaternion, vector: Vector) -> Vector:
    # q * [vector, 0] * q^-1, written without constructing temporary objects.
    qx, qy, qz, qw = _normalise_quaternion(q)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _compose(first: RigidTransform, second: RigidTransform) -> RigidTransform:
    """Return first * second, using T_A_C = T_A_B * T_B_C."""
    first_translation, first_rotation = first
    second_translation, second_rotation = second
    rotated_translation = _rotate(first_rotation, second_translation)
    return (
        tuple(
            first_translation[index] + rotated_translation[index]
            for index in range(3)
        ),
        _quaternion_multiply(first_rotation, second_rotation),
    )  # type: ignore[return-value]


def _inverse(transform: RigidTransform) -> RigidTransform:
    translation, rotation = transform
    inverse_rotation = _quaternion_inverse(rotation)
    inverse_translation = _rotate(
        inverse_rotation, (-translation[0], -translation[1], -translation[2])
    )
    return (inverse_translation, inverse_rotation)


def _from_odometry_pose(message: Odometry) -> RigidTransform:
    pose = message.pose.pose
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        _normalise_quaternion((
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )),
    )


def _from_transform_message(message: TransformStamped) -> RigidTransform:
    transform = message.transform
    return (
        (transform.translation.x, transform.translation.y, transform.translation.z),
        _normalise_quaternion((
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )),
    )


def _diagonal_covariance(variances: List[float]) -> List[float]:
    covariance = [0.0] * 36
    for index, variance in enumerate(variances):
        covariance[index * 6 + index] = max(float(variance), 1.0e-9)
    return covariance


class OkvisOdomAdapter(Node):
    """Convert private-frame OKVIS poses to aligned base poses without TF."""

    def __init__(self) -> None:
        super().__init__("okvis_odom_adapter")

        self.okvis_odom_topic = self.declare_parameter(
            "okvis_odom_topic", "/okvis/okvis_odometry"
        ).value
        self.wheel_odom_topic = self.declare_parameter(
            "wheel_odom_topic", "/target/odometry/filtered"
        ).value
        self.aligned_odom_topic = self.declare_parameter(
            "aligned_odom_topic", "/target/odometry/okvis_aligned"
        ).value
        self.odom_frame = self.declare_parameter(
            "odom_frame", "target/odom"
        ).value
        self.base_frame = self.declare_parameter(
            "base_frame", "target/base_link"
        ).value
        self.tracking_frame = self.declare_parameter(
            "tracking_frame", "target/rs_camera"
        ).value
        self.tf_lookup_timeout = float(self.declare_parameter(
            "tf_lookup_timeout_sec", 0.2
        ).value)
        self.max_alignment_time_difference = float(self.declare_parameter(
            "max_alignment_time_difference_sec", 0.5
        ).value)
        self.use_input_covariance = bool(self.declare_parameter(
            "use_input_covariance", False
        ).value)
        pose_variances = list(self.declare_parameter(
            "pose_variances", [0.05, 0.05, 1.0, 1.0, 1.0, 0.05]
        ).value)
        if len(pose_variances) != 6:
            self.get_logger().warning(
                "pose_variances must contain six values; using conservative defaults"
            )
            pose_variances = [0.05, 0.05, 1.0, 1.0, 1.0, 0.05]
        self.pose_covariance = _diagonal_covariance(pose_variances)
        self.twist_covariance = _diagonal_covariance([1.0e6] * 6)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.aligned_publisher = self.create_publisher(
            Odometry, self.aligned_odom_topic, 10
        )
        self.wheel_subscription = self.create_subscription(
            Odometry, self.wheel_odom_topic, self._wheel_callback, 20
        )
        self.okvis_subscription = self.create_subscription(
            Odometry, self.okvis_odom_topic, self._okvis_callback, 20
        )

        self.wheel_history: Deque[Odometry] = deque(maxlen=200)
        self.odom_to_okvis: Optional[RigidTransform] = None
        self.base_to_tracking: Optional[RigidTransform] = None
        self.last_warning_ns = 0

        self.get_logger().info(
            f"Aligning {self.okvis_odom_topic} to {self.odom_frame} using "
            f"{self.wheel_odom_topic}; publishing pose-only "
            f"{self.aligned_odom_topic} without TF"
        )

    def _warn_throttled(self, message: str, period_sec: float = 5.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= int(period_sec * 1.0e9):
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def _wheel_callback(self, message: Odometry) -> None:
        self.wheel_history.append(message)

    def _wheel_sample_for(self, stamp) -> Optional[Odometry]:
        if not self.wheel_history:
            return None
        target_time = _stamp_seconds(stamp)
        return min(
            self.wheel_history,
            key=lambda message: abs(_stamp_seconds(message.header.stamp) - target_time),
        )

    def _lookup_base_to_tracking(self) -> Optional[RigidTransform]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tracking_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
        except TransformException as exception:
            self._warn_throttled(
                f"Waiting for static TF {self.base_frame} -> "
                f"{self.tracking_frame}: {exception}"
            )
            return None
        return _from_transform_message(transform)

    def _try_initial_alignment(self, okvis_message: Odometry) -> bool:
        wheel_message = self._wheel_sample_for(okvis_message.header.stamp)
        if wheel_message is None:
            self._warn_throttled(
                "Waiting for a wheel-odometry sample before aligning OKVIS"
            )
            return False

        time_difference = abs(
            _stamp_seconds(wheel_message.header.stamp)
            - _stamp_seconds(okvis_message.header.stamp)
        )
        if time_difference > self.max_alignment_time_difference:
            self._warn_throttled(
                "Wheel and OKVIS timestamps are too far apart for initial alignment "
                f"({time_difference:.3f}s)"
            )
            return False

        base_to_tracking = self._lookup_base_to_tracking()
        if base_to_tracking is None:
            return False

        odom_to_base = _from_odometry_pose(wheel_message)
        odom_to_tracking = _compose(odom_to_base, base_to_tracking)
        okvis_to_tracking = _from_odometry_pose(okvis_message)
        self.odom_to_okvis = _compose(odom_to_tracking, _inverse(okvis_to_tracking))
        self.base_to_tracking = base_to_tracking
        self.get_logger().info(
            "Initial OKVIS alignment established using "
            f"{time_difference:.3f} s-separated wheel/OKVIS samples"
        )
        return True

    def _okvis_callback(self, message: Odometry) -> None:
        if message.child_frame_id and message.child_frame_id != self.tracking_frame:
            self._warn_throttled(
                f"OKVIS child frame is '{message.child_frame_id}', expected "
                f"'{self.tracking_frame}'; dropping measurement"
            )
            return

        if self.odom_to_okvis is None and not self._try_initial_alignment(message):
            return
        if self.odom_to_okvis is None:
            return
        if self.base_to_tracking is None:
            self.base_to_tracking = self._lookup_base_to_tracking()
        if self.base_to_tracking is None:
            return

        okvis_to_tracking = _from_odometry_pose(message)
        odom_to_tracking = _compose(self.odom_to_okvis, okvis_to_tracking)
        odom_to_base = _compose(odom_to_tracking, _inverse(self.base_to_tracking))

        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = odom_to_base[0][0]
        output.pose.pose.position.y = odom_to_base[0][1]
        output.pose.pose.position.z = odom_to_base[0][2]
        output.pose.pose.orientation.x = odom_to_base[1][0]
        output.pose.pose.orientation.y = odom_to_base[1][1]
        output.pose.pose.orientation.z = odom_to_base[1][2]
        output.pose.pose.orientation.w = odom_to_base[1][3]
        if self.use_input_covariance and any(
            value > 0.0 for value in message.pose.covariance
        ):
            output.pose.covariance = list(message.pose.covariance)
        else:
            output.pose.covariance = list(self.pose_covariance)
        # This is intentionally a pose-only measurement. The OKVIS twist is
        # expressed at the camera/IMU tracking frame and is not used by the EKF.
        output.twist.covariance = list(self.twist_covariance)
        self.aligned_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OkvisOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
