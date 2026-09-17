#!/usr/bin/env python3
"""Publish NavSatFix measurements only while the F9P reports RTK fixed."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import CarrSoln, UBXNavPVT


MILLIMETERS_PER_METER = 1000.0


class RtkFixFilter(Node):
    """Gate the high-precision fix with the receiver's carrier solution state."""

    def __init__(self) -> None:
        super().__init__("rtk_fix_filter")
        self.input_fix_topic = str(self.declare_parameter(
            "input_fix_topic", "/gps/fix"
        ).value)
        self.input_pvt_topic = str(self.declare_parameter(
            "input_pvt_topic", "/gps/ubx_nav_pvt"
        ).value)
        self.output_fix_topic = str(self.declare_parameter(
            "output_fix_topic", "/target/gps/fix/rtk"
        ).value)
        self.require_rtk_fixed = bool(self.declare_parameter(
            "require_rtk_fixed", True
        ).value)
        self.maximum_horizontal_accuracy_m = float(self.declare_parameter(
            "maximum_horizontal_accuracy_m", 0.25
        ).value)
        self.maximum_pvt_age_sec = float(self.declare_parameter(
            "maximum_pvt_age_sec", 0.50
        ).value)
        if self.maximum_horizontal_accuracy_m <= 0.0:
            raise ValueError("maximum_horizontal_accuracy_m must be positive")
        if self.maximum_pvt_age_sec <= 0.0:
            raise ValueError("maximum_pvt_age_sec must be positive")

        self.latest_pvt: Optional[UBXNavPVT] = None
        self.latest_pvt_receive_ns: Optional[int] = None
        self.last_rejection = ""
        self.last_warning_ns = 0

        self.publisher = self.create_publisher(
            NavSatFix, self.output_fix_topic, qos_profile_sensor_data
        )
        self.pvt_subscription = self.create_subscription(
            UBXNavPVT,
            self.input_pvt_topic,
            self._pvt_callback,
            qos_profile_sensor_data,
        )
        self.fix_subscription = self.create_subscription(
            NavSatFix,
            self.input_fix_topic,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Gating {self.input_fix_topic} with {self.input_pvt_topic}; "
            f"publishing accepted RTK fixes on {self.output_fix_topic}"
        )

    def _pvt_callback(self, message: UBXNavPVT) -> None:
        self.latest_pvt = message
        self.latest_pvt_receive_ns = self.get_clock().now().nanoseconds

    def _reject_reason(self, fix: NavSatFix) -> Optional[str]:
        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return "NavSatFix reports no fix"
        if not all(math.isfinite(value) for value in (
            fix.latitude, fix.longitude, fix.altitude
        )):
            return "NavSatFix contains non-finite coordinates"
        if self.latest_pvt is None or self.latest_pvt_receive_ns is None:
            return "UBX-NAV-PVT has not been received"

        pvt_age = (
            self.get_clock().now().nanoseconds - self.latest_pvt_receive_ns
        ) * 1.0e-9
        if pvt_age > self.maximum_pvt_age_sec:
            return f"UBX-NAV-PVT is stale ({pvt_age:.2f}s)"

        pvt = self.latest_pvt
        if not pvt.gnss_fix_ok or int(pvt.gps_fix.fix_type) < 3 or pvt.invalid_llh:
            return "UBX-NAV-PVT does not report a valid 3D GNSS fix"
        if self.require_rtk_fixed and int(pvt.carr_soln.status) != int(
            CarrSoln.CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES
        ):
            return "carrier solution is not RTK fixed"

        horizontal_accuracy_m = float(pvt.h_acc) / MILLIMETERS_PER_METER
        if horizontal_accuracy_m > self.maximum_horizontal_accuracy_m:
            return (
                f"horizontal accuracy {horizontal_accuracy_m:.3f}m exceeds "
                f"{self.maximum_horizontal_accuracy_m:.3f}m"
            )
        return None

    def _warn_rejection(self, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if reason != self.last_rejection or now_ns - self.last_warning_ns >= 5_000_000_000:
            self.get_logger().warning(f"Suppressing GNSS fix: {reason}")
            self.last_rejection = reason
            self.last_warning_ns = now_ns

    def _fix_callback(self, message: NavSatFix) -> None:
        reason = self._reject_reason(message)
        if reason is not None:
            self._warn_rejection(reason)
            return
        if self.last_rejection:
            self.get_logger().info("RTK fix accepted; resuming GNSS output")
            self.last_rejection = ""
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkFixFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
