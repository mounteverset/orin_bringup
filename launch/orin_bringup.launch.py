#!/usr/bin/env python3
"""Unified Orin bringup for sensors, SLAM, Nav2, OKVIS, and generalist_bt_gen."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _expanded_path(value):
    return os.path.expandvars(os.path.expanduser(str(value)))


def _as_bool(value, key):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"{key} must be true or false, got {value!r}")


def _enabled(context, launch_argument, configured_value):
    override = LaunchConfiguration(launch_argument).perform(context).strip().lower()
    return _as_bool(configured_value if override == "auto" else override, launch_argument)


def _require_file(path, description):
    if not Path(path).is_file():
        raise RuntimeError(f"Missing {description}: {path}")


def _launch_setup(context):
    config_file = _expanded_path(LaunchConfiguration("config").perform(context))
    _require_file(config_file, "bringup config")
    nav2_config_value = LaunchConfiguration("nav2_config").perform(context).strip()
    nav2_config_file = (
        _expanded_path(nav2_config_value) if nav2_config_value else None
    )
    if nav2_config_file:
        _require_file(nav2_config_file, "Nav2 mode config")
    with open(config_file, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    try:
        cfg = document["orin_bringup"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"{config_file} must contain orin_bringup.ros__parameters"
        ) from exc

    subsystems = cfg["subsystems"]
    enable_nav2 = _enabled(context, "enable_nav2", subsystems["nav2"])
    enable_slam = _enabled(context, "enable_slam", subsystems["slam_toolbox"])
    enable_livox = _enabled(context, "enable_livox", subsystems["livox"])
    enable_ublox = _enabled(context, "enable_ublox", subsystems["ublox"])
    enable_f9p_heading_adapter = _enabled(
        context, "enable_f9p_heading_adapter", subsystems["f9p_heading_adapter"]
    )
    enable_navsat_transform = _enabled(
        context, "enable_navsat_transform", subsystems["navsat_transform"]
    )
    enable_okvis = _enabled(context, "enable_okvis", subsystems["okvis_findanything"])
    enable_generalist = _enabled(
        context, "enable_generalist", subsystems["generalist_bt_gen"]
    )
    enable_odom_fusion = _enabled(
        context, "enable_odom_fusion", subsystems["odom_fusion"]
    )
    enable_gps_global_fusion = _enabled(
        context, "enable_gps_global_fusion", subsystems["gps_global_fusion"]
    )

    if enable_slam and enable_gps_global_fusion:
        raise RuntimeError(
            "SLAM Toolbox and the GPS global EKF would both publish map->odom. "
            "Enable exactly one global localization provider."
        )
    if enable_gps_global_fusion and not enable_odom_fusion:
        raise RuntimeError(
            "GPS global fusion requires enable_odom_fusion:=true so a separate "
            "continuous odom->base_link transform exists."
        )
    if enable_gps_global_fusion and not enable_navsat_transform:
        raise RuntimeError(
            "GPS global fusion requires enable_navsat_transform:=true."
        )

    network = cfg["network"]
    frames = cfg["frames"]
    topics = cfg["topics"]
    runtime = cfg["runtime"]
    use_sim_time = _as_bool(runtime["use_sim_time"], "runtime.use_sim_time")
    respawn = _as_bool(runtime["respawn"], "runtime.respawn")
    log_level = str(runtime["log_level"])
    common_node_options = {
        "output": "screen",
        "respawn": respawn,
        "respawn_delay": float(runtime["respawn_delay"]),
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    # generalist_bt_gen overlays BehaviorTree.CPP 4.8.2, while apt Nav2 1.3.12
    # is linked against 4.9.0. Keep the system library first for Nav2 processes.
    nav2_environment = {
        "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
    }

    actions = [
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", str(network["rmw_implementation"])
        ),
        SetEnvironmentVariable(
            "CYCLONEDDS_URI", f"file://{_expanded_path(network['cyclonedds_config'])}"
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(network["ros_domain_id"])),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", str(network["discovery_range"])
        ),
        UnsetEnvironmentVariable("ROS_LOCALHOST_ONLY"),
        LogInfo(
            msg=(
                "Orin unified bringup: domain "
                f"{network['ros_domain_id']}, map/odom/base frames "
                f"{frames['map']}/{frames['odom']}/{frames['base']}"
            )
        ),
    ]
    if enable_generalist and not enable_slam:
        actions.append(
            LogInfo(
                msg=(
                    "Mapless mode: FindAnything coordinates and direct navigation remain "
                    "available, but Generalist's annotated-SLAM-map context is unavailable."
                )
            )
        )

    if enable_livox:
        livox_config = _expanded_path(cfg["livox"]["user_config_path"])
        _require_file(livox_config, "Livox MID-360 JSON config")
        actions.extend(
            [
                Node(
                    package="livox_ros_driver2",
                    executable="livox_ros_driver2_node",
                    name="livox_lidar_publisher",
                    parameters=[
                        config_file,
                        {
                            "frame_id": frames["lidar"],
                            "user_config_path": livox_config,
                        },
                    ],
                    remappings=[("/livox/lidar", topics["pointcloud"])],
                    **common_node_options,
                ),
                Node(
                    package="pointcloud_to_laserscan",
                    executable="pointcloud_to_laserscan_node",
                    name="livox_pointcloud_to_laserscan",
                    parameters=[config_file, {"target_frame": frames["lidar"]}],
                    remappings=[
                        ("cloud_in", topics["pointcloud"]),
                        ("scan", topics["scan"]),
                    ],
                    **common_node_options,
                ),
            ]
        )

    static_tf = cfg["static_transforms"]
    if _as_bool(static_tf["publish_base_to_lidar"], "publish_base_to_lidar"):
        xyz_rpy = static_tf["base_to_lidar_xyz_rpy"]
        if len(xyz_rpy) != 6:
            raise RuntimeError("base_to_lidar_xyz_rpy must contain [x,y,z,roll,pitch,yaw]")
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_static_tf",
                arguments=[
                    "--x", str(xyz_rpy[0]), "--y", str(xyz_rpy[1]),
                    "--z", str(xyz_rpy[2]), "--roll", str(xyz_rpy[3]),
                    "--pitch", str(xyz_rpy[4]), "--yaw", str(xyz_rpy[5]),
                    "--frame-id", frames["base"],
                    "--child-frame-id", frames["lidar"],
                ],
                output="screen",
            )
        )

    if enable_ublox:
        ublox_launch = Path(get_package_share_directory("husky_gnss")) / "launch" / "husky_f9p.launch.py"
        ublox = cfg["ublox"]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ublox_launch)),
                launch_arguments={
                    "namespace": str(ublox["namespace"]),
                    "frame_id": str(frames["gps"]),
                    "device_serial_string": str(ublox["device_serial_string"]),
                    "measurement_period_ms": str(ublox["measurement_period_ms"]),
                    "enable_ntrip": str(ublox["enable_ntrip"]).lower(),
                    "ntrip_use_https": str(ublox["ntrip_use_https"]).lower(),
                    "ntrip_host": str(ublox["ntrip_host"]),
                    "ntrip_port": str(ublox["ntrip_port"]),
                    "ntrip_mountpoint": str(ublox["ntrip_mountpoint"]),
                    "respawn": str(respawn).lower(),
                    "log_level": log_level,
                }.items(),
            )
        )

    if enable_f9p_heading_adapter:
        actions.append(
            Node(
                package="orin_bringup",
                executable="f9p_course_heading_adapter",
                name="f9p_course_heading_adapter",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": topics["ubx_nav_pvt"],
                        "odom_topic": topics["fused_odom"],
                        "output_topic": topics["navsat_imu"],
                        "output_frame": frames["base"],
                    },
                ],
                **common_node_options,
            )
        )

    if enable_navsat_transform:
        # robot_localization prints the current datum at INFO every time it
        # retries the transform. Keep warnings and errors, but suppress its
        # repetitive informational output.
        navsat_node_options = {
            **common_node_options,
            "arguments": ["--ros-args", "--log-level", "warn"],
        }
        actions.append(
            Node(
                package="orin_bringup",
                executable="rtk_fix_filter",
                name="rtk_fix_filter",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "input_fix_topic": topics["raw_gps_fix"],
                        "input_pvt_topic": topics["ubx_nav_pvt"],
                        "output_fix_topic": topics["gps_fix"],
                    },
                ],
                **common_node_options,
            )
        )
        actions.append(
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("gps/fix", topics["gps_fix"]),
                    ("imu", topics["navsat_imu"]),
                    (
                        "odometry/filtered",
                        topics["global_odom"]
                        if enable_gps_global_fusion
                        else topics["fused_odom"],
                    ),
                    ("odometry/gps", topics["gps_odom"]),
                    ("gps/filtered", topics["filtered_gps"]),
                ],
                **navsat_node_options,
            )
        )

    if enable_slam:
        slam = cfg["slam"]
        slam_mode_override = LaunchConfiguration("slam_mode").perform(context).strip()
        slam_mode = str(
            slam["mode"] if slam_mode_override == "auto" else slam_mode_override
        ).lower()
        if slam_mode not in ("mapping", "localization"):
            raise RuntimeError(
                f"slam_mode must be mapping or localization, got {slam_mode!r}"
            )
        slam_map_file = _expanded_path(
            LaunchConfiguration("slam_map_file").perform(context).strip()
        )
        if slam_mode == "localization" and not slam_map_file:
            raise RuntimeError(
                "slam_mode:=localization requires slam_map_file:=<serialized-map-basename>"
            )
        executable = (
            str(slam["mapping_executable"])
            if slam_mode == "mapping"
            else str(slam["localization_executable"])
        )
        slam_overrides = {
            "use_sim_time": use_sim_time,
            "map_frame": frames["map"],
            "odom_frame": frames["odom"],
            "base_frame": frames["base"],
            "scan_topic": topics["scan"],
            "mode": slam_mode,
        }
        if slam_map_file:
            slam_overrides["map_file_name"] = slam_map_file
        actions.extend(
            [
                LifecycleNode(
                    package="slam_toolbox",
                    executable=executable,
                    name="slam_toolbox",
                    namespace="",
                    parameters=[
                        config_file,
                        slam_overrides,
                    ],
                    **common_node_options,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_slam",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "autostart": True,
                            "bond_timeout": 0.0,
                            "node_names": ["slam_toolbox"],
                        }
                    ],
                ),
            ]
        )

    if enable_nav2:
        nav_nodes = [
            ("nav2_controller", "controller_server", "controller_server"),
            ("nav2_smoother", "smoother_server", "smoother_server"),
            ("nav2_planner", "planner_server", "planner_server"),
            ("nav2_behaviors", "behavior_server", "behavior_server"),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        ]
        for package, executable, name in nav_nodes:
            remappings = []
            if name in ("controller_server", "behavior_server"):
                remappings.append(("cmd_vel", topics["nav_cmd_vel"]))
            nav_parameters = [config_file]
            if nav2_config_file:
                nav_parameters.append(nav2_config_file)
            nav_parameters.append({"use_sim_time": use_sim_time})
            actions.append(
                Node(
                    package=package,
                    executable=executable,
                    name=name,
                    parameters=nav_parameters,
                    remappings=remappings,
                    additional_env=nav2_environment,
                    **common_node_options,
                )
            )

        actions.append(
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cmd_vel", topics["nav_cmd_vel"]),
                    ("cmd_vel_smoothed", topics["base_cmd_vel"]),
                ],
                additional_env=nav2_environment,
                **common_node_options,
            )
        )
        lifecycle_nodes = [name for _, _, name in nav_nodes] + ["velocity_smoother"]
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "bond_timeout": float(cfg["nav2"]["bond_timeout"]),
                        "attempt_respawn_reconnection": respawn,
                        "node_names": lifecycle_nodes,
                    }
                ],
            )
        )

    if enable_okvis:
        okvis = cfg["okvis"]
        okvis_launch = _expanded_path(okvis["launch_file"])
        rsusb_lib = _expanded_path(okvis["rsusb_library_path"])
        torch_lib = _expanded_path(okvis["torch_library_path"])
        _require_file(okvis_launch, "OKVIS ROS 2 launch file")
        _require_file(Path(rsusb_lib) / "librealsense2.so", "OKVIS RSUSB library")
        _require_file(Path(torch_lib) / "libtorch.so", "OKVIS LibTorch library")
        okvis_ld_path = f"{rsusb_lib}:{torch_lib}:" + os.environ.get("LD_LIBRARY_PATH", "")
        actions.append(
            GroupAction(
                scoped=True,
                actions=[
                    SetEnvironmentVariable("LD_LIBRARY_PATH", okvis_ld_path),
                    SetEnvironmentVariable("OKVIS_HEADLESS", "1"),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(okvis_launch),
                        launch_arguments={
                            "config_filename": _expanded_path(okvis["config_filename"]),
                            "se_config_filename": _expanded_path(okvis["se_config_filename"]),
                            "vision_language": str(okvis["vision_language"]).lower(),
                            "language_query_node": str(okvis["language_query_node"]).lower(),
                            "clip_mesh_below_floor": str(
                                okvis["clip_mesh_below_floor"]
                            ).lower(),
                            "mesh_floor_z": str(okvis["mesh_floor_z"]),
                            "object_location_target_frame": frames["map"],
                            "navigation_frame_id": str(okvis["navigation_frame_id"]),
                            "estimator_frame_id": str(okvis["estimator_frame_id"]),
                            "tracking_frame_id": str(okvis["tracking_frame_id"]),
                            "publish_navigation_alignment": str(
                                okvis["publish_navigation_alignment"]
                            ).lower(),
                            "rviz": "false",
                        }.items(),
                    ),
                ],
            )
        )

    if enable_odom_fusion:
        fusion = cfg["odom_fusion"]
        use_livox_imu = _as_bool(
            fusion["use_livox_imu"], "odom_fusion.use_livox_imu"
        )
        use_okvis_pose = _as_bool(
            fusion.get("use_okvis_pose", False), "odom_fusion.use_okvis_pose"
        )
        adapter_overrides = {
            "use_sim_time": use_sim_time,
            "okvis_odom_topic": topics["okvis_odom"],
            "wheel_odom_topic": topics["odom"],
            "aligned_odom_topic": topics["okvis_aligned_odom"],
            "odom_frame": frames["odom"],
            "base_frame": frames["base"],
            "tracking_frame": str(cfg["okvis"]["tracking_frame_id"]),
            "tf_lookup_timeout_sec": float(fusion["tf_lookup_timeout_sec"]),
            "max_alignment_time_difference_sec": float(
                fusion["max_alignment_time_difference_sec"]
            ),
            "pose_variances": list(fusion["pose_variances"]),
        }
        ekf_overrides = {
            "use_sim_time": use_sim_time,
            "imu0": topics["imu"],
            "odom0": topics["wheel_fusion_odom"],
            "odom1": (
                topics["okvis_aligned_odom"]
                if use_okvis_pose
                else "/unused/okvis_aligned"
            ),
        }
        map_ekf_overrides = {
            "use_sim_time": use_sim_time,
            "imu0": topics["imu"],
            "odom0": topics["wheel_fusion_odom"],
            "odom1": topics["gps_odom"],
            "odom2": (
                topics["okvis_aligned_odom"]
                if use_okvis_pose
                else "/unused/okvis_aligned_global"
            ),
        }
        fusion_actions = []
        fusion_actions.append(
            Node(
                package="orin_bringup",
                executable="wheel_odom_covariance_adapter",
                name="wheel_odom_covariance_adapter",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": topics["odom"],
                        "output_topic": topics["wheel_fusion_odom"],
                        "linear_x_variance": float(fusion["wheel_linear_x_variance"]),
                        "angular_z_variance": float(fusion["wheel_angular_z_variance"]),
                    },
                ],
                **common_node_options,
            )
        )
        if use_livox_imu:
            fusion_actions.append(
                Node(
                    package="orin_bringup",
                    executable="livox_imu_adapter",
                    name="livox_imu_adapter",
                    parameters=[
                        config_file,
                        {
                            "use_sim_time": use_sim_time,
                            "input_topic": topics["livox_imu"],
                            "output_topic": topics["imu"],
                            "output_frame": frames["lidar"],
                        },
                    ],
                    **common_node_options,
                )
            )
        if use_okvis_pose:
            fusion_actions.append(
                Node(
                    package="orin_bringup",
                    executable="okvis_odom_adapter",
                    name="okvis_odom_adapter",
                    parameters=[config_file, adapter_overrides],
                    **common_node_options,
                )
            )
        fusion_actions.extend(
            [
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_odom_fusion",
                    parameters=[config_file, ekf_overrides],
                    remappings=[
                        ("odometry/filtered", topics["fused_odom"]),
                    ],
                    **common_node_options,
                ),
                Node(
                    package="orin_bringup",
                    executable="fused_odom_trail_marker",
                    name="fused_odom_trail_marker",
                    parameters=[
                        config_file,
                        {
                            "use_sim_time": use_sim_time,
                            "input_topic": topics["fused_odom"],
                            "output_topic": topics["fused_trail_marker"],
                            "frame_id": frames["odom"],
                            "publish_frequency_hz": 100.0,
                        },
                    ],
                    **common_node_options,
                ),
            ]
        )
        if enable_gps_global_fusion:
            fusion_actions.append(
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_map_fusion",
                    parameters=[config_file, map_ekf_overrides],
                    remappings=[
                        ("odometry/filtered", topics["global_odom"]),
                    ],
                    **common_node_options,
                )
            )
        actions.extend(fusion_actions)


    if enable_generalist:
        generalist = cfg["generalist"]
        generalist_launch = (
            Path(get_package_share_directory(str(generalist["package"])))
            / "launch"
            / str(generalist["launch_file"])
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(generalist_launch)),
                launch_arguments={
                    "use_cli_ui": str(generalist["use_cli_ui"]).lower(),
                    "demo_mode": str(generalist["demo_mode"]).lower(),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("orin_bringup"), "config", "orin_bringup.yaml"
    )
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("nav2_config", default_value=""),
        DeclareLaunchArgument("enable_nav2", default_value="auto"),
        DeclareLaunchArgument("enable_slam", default_value="auto"),
        DeclareLaunchArgument("enable_livox", default_value="auto"),
        DeclareLaunchArgument("enable_ublox", default_value="auto"),
        DeclareLaunchArgument("enable_f9p_heading_adapter", default_value="auto"),
        DeclareLaunchArgument("enable_navsat_transform", default_value="auto"),
        DeclareLaunchArgument("enable_okvis", default_value="auto"),
        DeclareLaunchArgument("enable_generalist", default_value="auto"),
        DeclareLaunchArgument("enable_odom_fusion", default_value="auto"),
        DeclareLaunchArgument("enable_gps_global_fusion", default_value="auto"),
        DeclareLaunchArgument("slam_mode", default_value="auto"),
        DeclareLaunchArgument("slam_map_file", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
