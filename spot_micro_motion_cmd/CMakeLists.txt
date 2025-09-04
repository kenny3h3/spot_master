cmake_minimum_required(VERSION 3.8)
project(spot_micro_motion_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(i2cpwm_board_msgs REQUIRED)
find_package(smov_msgs REQUIRED)
find_package(smov_lib REQUIRED)  <!-- Ersetzt spot_micro_kinematics -->
find_package(smov_config REQUIRED)

include_directories(include)

add_executable(spot_micro_motion_cmd_node
  src/spot_micro_motion_cmd.cpp
)

ament_target_dependencies(spot_micro_motion_cmd_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  i2cpwm_board_msgs
  smov_msgs
  smov_lib  <!-- Ersetzt spot_micro_kinematics -->
  smov_config
)

install(TARGETS
  spot_micro_motion_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  i2cpwm_board_msgs
  smov_msgs
  smov_lib  <!-- Ersetzt spot_micro_kinematics -->
  smov_config
)

ament_package()
