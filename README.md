# orin_bringup

Unified ROS 2 Jazzy bringup for the Jetson Orin. The default configuration is
mapless outdoor navigation using RTK GNSS, wheel odometry, the Livox IMU, live
LiDAR obstacle avoidance, OKVIS/FindAnything, Nav2, and `generalist_bt_gen`.
SLAM Toolbox remains in the launch file but is disabled by default.

## Default localization graph

The two `robot_localization` filters have separate responsibilities:

```text
wheel twist + Livox yaw rate (+ optional OKVIS pose)
                     |
                     v
             ekf_odom_fusion -------- target/odom -> target/base_link
                     |
RTK fix -> RTK gate -> navsat_transform -> ekf_map_fusion -> map -> odom
```

`ekf_odom_fusion` provides continuous local odometry on
`/target/odometry/fused`. `ekf_map_fusion` adds
`/target/odometry/gps`, publishes `/target/odometry/global`, and is the only
default owner of `target/map -> target/odom`.

The raw `/gps/fix` stream is filtered through the UBX-NAV-PVT carrier state.
Only RTK-fixed measurements meeting the configured horizontal-accuracy limit
are forwarded on `/target/gps/fix/rtk` to `navsat_transform_node`.

OKVIS remains independent in `target/okvis_odom`. Its alignment publisher owns
only `target/odom -> target/okvis_odom`, so FindAnything locations and meshes
can be transformed through the GPS-localized `target/map` frame without feeding
OKVIS into either EKF. Optional VIO fusion remains disabled with
`odom_fusion.use_okvis_pose: false`.

## Build and choose a navigation mode

```bash
source /opt/ros/jazzy/setup.bash
source /home/orin/livox_ros2_ws/install/setup.bash
source /home/orin/ublox_ws/install/setup.bash
source /home/orin/okvis_ws/install/setup.bash
source /home/orin/generalist_bt_gen/install/setup.bash
cd /home/orin/orin_bringup_ws
colcon build --symlink-install
source install/setup.bash
```

For outdoor RTK navigation without a prebuilt map:

```bash
ros2 launch orin_bringup gps_navigation.launch.py
```

For indoor SLAM while creating or extending a map:

```bash
ros2 launch orin_bringup slam_navigation.launch.py
```

When the map is ready, serialize the pose graph (the service creates the
`.posegraph` and `.data` pair):

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/absolute/path/to/site_map'}"
```

For indoor localization and navigation on a previously serialized SLAM
Toolbox pose graph:

```bash
ros2 launch orin_bringup slam_navigation.launch.py \
  slam_mode:=localization \
  slam_map_file:=/absolute/path/to/site_map
```

`slam_map_file` is the basename used by SLAM Toolbox serialization (normally
the path without `.posegraph` or `.data`). Stop the running mode before
starting the other one; the launchers intentionally select different sole
publishers for `target/map -> target/odom`.

Both launchers keep the local wheel/IMU EKF, OKVIS, FindAnything, Livox, Nav2,
and Generalist settings from the common config. The GPS launcher starts the
F9P pipeline and uses a rolling mapless global costmap. The SLAM launcher stops
the GPS localization pipeline and uses the SLAM occupancy grid as a static
global costmap. Meshes therefore remain visible in RViz in either mode as long
as the complete TF chain is available.

For a sensor/fusion diagnostic run without Nav2 or Generalist:

```bash
ros2 launch orin_bringup gps_navigation.launch.py \
  enable_nav2:=false enable_generalist:=false
```

## TF ownership contract

Exactly one publisher may own each dynamic TF edge:

- `ekf_odom_fusion`: `target/odom -> target/base_link`
- GPS mode, `ekf_map_fusion`: `target/map -> target/odom`
- SLAM mode, `slam_toolbox`: `target/map -> target/odom`
- OKVIS alignment: `target/odom -> target/okvis_odom`

Disable the Humble base's existing `target/odom -> target/base_link` broadcaster
before using the default configuration. The launch rejects configurations that
enable both SLAM Toolbox and the GPS global EKF because both would publish
`target/map -> target/odom`.

Static transforms must exist from `target/base_link` to the LiDAR, GNSS antenna,
and RealSense tracking frames. The optional base-to-LiDAR publisher is disabled
unless explicitly configured with measured extrinsics.

Both computers must use synchronized clocks (chrony or PTP), the same
`ROS_DOMAIN_ID`, and compatible Cyclone DDS networking.

## RTK initialization

The single-antenna F9P provides course rather than stationary vehicle heading.
`f9p_course_heading_adapter` waits for a sufficiently fast, straight, forward
segment before giving `navsat_transform_node` an ENU heading. The robot may need
a short operator-controlled initialization drive before GPS navigation becomes
available. The configured speed, heading-accuracy, and consistency gates must
be validated against recorded F9P data.

`wait_for_datum` is currently false, so the first accepted fix establishes the
origin on each startup. Configure a fixed datum before relying on persistent
geographic object locations or repeatable Cartesian coordinates across runs.

## Mapless Nav2 behavior

The global costmap is a 200 m square rolling window using the live `/scan`
obstacle layer. Its static layer is retained but disabled, and unknown space is
treated as traversable because no occupancy map exists. Split GPS routes into
legs shorter than roughly half the costmap width or enlarge the window with due
attention to memory use.

Mapless navigation only knows obstacles currently observed by the sensors. It
does not know distant buildings, fences, keep-out areas, or road topology. Use
GPS waypoint routes, OpenStreetMap constraints, or explicit keep-out zones for
structured outdoor missions.

FindAnything itself does not require `/map`: it transforms persistent OKVIS
object positions from `target/okvis_odom` into `target/map`. The current
Generalist `find_and_drive_to_nearest_object` planning metadata, however, still
requests an annotated SLAM `/map` image. Raw object queries and direct Nav2
goals work, but that particular LLM planning workflow must be changed to use a
satellite map or remove its annotated-SLAM-map requirement.

## Advanced unified launch

SLAM code and parameters remain in the shared launch. The two mode launchers
above only set safe combinations of its arguments. For manual testing, the
equivalent SLAM selection is:

```bash
ros2 launch orin_bringup orin_bringup.launch.py \
  enable_slam:=true enable_gps_global_fusion:=false
```

The mode launchers additionally apply their respective costmap overlays from
`config/gps_navigation.yaml` and `config/slam_navigation.yaml`; the common YAML
does not need to be edited when switching modes.

## Before enabling autonomous motion

Verify publishers, timestamps, corrections, and the complete TF chain:

```bash
ros2 topic info -v /target/odometry/filtered
ros2 topic info -v /target/odometry/fused
ros2 topic info -v /target/odometry/gps
ros2 topic info -v /target/odometry/global
ros2 topic echo /gps/fix --once
ros2 topic echo /target/gps/fix/rtk --once
ros2 run tf2_ros tf2_echo target/odom target/base_link
ros2 run tf2_ros tf2_echo target/map target/odom
ros2 run tf2_ros tf2_echo target/odom target/okvis_odom
ros2 run tf2_ros tf2_echo target/base_link target/livox_mount_link
```

Inspect `/scan`, both costmaps, the OKVIS mesh, RTK fix state, EKF diagnostics,
and FindAnything coordinates in RViz before sending a Nav2 goal. Use
`target/map` as the RViz fixed frame after GPS initialization; use
`target/odom` to inspect OKVIS while the global transform is not yet available.
