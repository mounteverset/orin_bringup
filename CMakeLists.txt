cmake_minimum_required(VERSION 3.8)
project(orin_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/okvis_odom_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME okvis_odom_adapter
)

install(PROGRAMS
  scripts/livox_imu_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME livox_imu_adapter
)

install(PROGRAMS
  scripts/f9p_course_heading_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME f9p_course_heading_adapter
)

install(PROGRAMS
  scripts/rtk_fix_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rtk_fix_filter
)

install(PROGRAMS
  scripts/wheel_odom_covariance_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odom_covariance_adapter
)

install(PROGRAMS
  scripts/fused_odom_trail_marker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fused_odom_trail_marker
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
