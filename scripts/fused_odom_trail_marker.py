#!/usr/bin/env python3
"""Publish a bounded RViz trail for fused odometry."""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker


class FusedOdomTrailMarker(Node):
    """Publish the latest fused path as a small RViz line marker."""

    def __init__(self) -> None:
        super().__init__("fused_odom_trail_marker")

        self.input_topic = str(self.declare_parameter(
            "input_topic", "/target/odometry/fused"
        ).value)
        self.output_topic = str(self.declare_parameter(
            "output_topic", "/target/odometry/fused_trail"
        ).value)
        self.frame_id = str(self.declare_parameter(
            "frame_id", "target/odom"
        ).value)
        self.publish_frequency_hz = float(self.declare_parameter(
            "publish_frequency_hz", 100.0
        ).value)
        self.max_points = int(self.declare_parameter(
            "max_points", 1000
        ).value)
        self.min_point_spacing = float(self.declare_parameter(
            "min_point_spacing", 0.02
        ).value)
        self.line_width = float(self.declare_parameter(
            "line_width", 0.04
        ).value)
        self.color_r = float(self.declare_parameter("color_r", 0.1).value)
        self.color_g = float(self.declare_parameter("color_g", 1.0).value)
        self.color_b = float(self.declare_parameter("color_b", 0.1).value)
        self.color_a = float(self.declare_parameter("color_a", 0.95).value)

        if self.publish_frequency_hz <= 0.0:
            raise ValueError("publish_frequency_hz must be positive")
        if self.max_points < 2:
            raise ValueError("max_points must be at least 2")
        if self.min_point_spacing < 0.0:
            raise ValueError("min_point_spacing cannot be negative")

        self.points = deque(maxlen=self.max_points)
        self.latest_frame_id = self.frame_id
        self.last_trail_point = None

        self.publisher = self.create_publisher(Marker, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._callback, 20
        )
        self.timer = self.create_timer(
            1.0 / self.publish_frequency_hz, self._publish
        )
        self.get_logger().info(
            f"Publishing {self.output_topic} at "
            f"{self.publish_frequency_hz:.1f} Hz from {self.input_topic}"
        )

    def _callback(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        z = float(message.pose.pose.position.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            return

        frame_id = self.frame_id or message.header.frame_id
        if not frame_id:
            return
        if self.latest_frame_id and self.latest_frame_id != frame_id:
            self.points.clear()
            self.last_trail_point = None
        self.latest_frame_id = frame_id

        point = Point()
        point.x = x
        point.y = y
        point.z = z
        if self.last_trail_point is None:
            self.points.append(point)
            self.last_trail_point = point
        elif math.hypot(
            point.x - self.last_trail_point.x, point.y - self.last_trail_point.y
        ) >= self.min_point_spacing:
            self.points.append(point)
            self.last_trail_point = point
        else:
            self.points[-1] = point

    def _publish(self) -> None:
        if not self.points or not self.latest_frame_id:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.latest_frame_id
        marker.ns = "fused_odom_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = self.color_a
        marker.points = list(self.points)
        self.publisher.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusedOdomTrailMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
