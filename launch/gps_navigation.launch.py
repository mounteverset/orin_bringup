#!/usr/bin/env python3
"""Outdoor mapless navigation with RTK GNSS as the global reference."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory("orin_bringup"))

    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value=str(package_share / "config" / "orin_bringup.yaml"),
            description="Common bringup configuration",
        ),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        DeclareLaunchArgument("enable_livox", default_value="auto"),
        DeclareLaunchArgument("enable_okvis", default_value="auto"),
        DeclareLaunchArgument("enable_generalist", default_value="auto"),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "orin_bringup.launch.py")
        ),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "nav2_config": str(
                package_share / "config" / "gps_navigation.yaml"
            ),
            "enable_nav2": LaunchConfiguration("enable_nav2"),
            "enable_slam": "false",
            "enable_livox": LaunchConfiguration("enable_livox"),
            "enable_ublox": "true",
            "enable_f9p_heading_adapter": "true",
            "enable_navsat_transform": "true",
            "enable_okvis": LaunchConfiguration("enable_okvis"),
            "enable_generalist": LaunchConfiguration("enable_generalist"),
            "enable_odom_fusion": "true",
            "enable_gps_global_fusion": "true",
        }.items(),
    )

    return LaunchDescription(arguments + [bringup])

