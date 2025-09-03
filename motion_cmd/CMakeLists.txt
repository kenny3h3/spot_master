cmake_minimum_required(VERSION 3.16)
project(motion_cmd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_lib REQUIRED)
find_package(smov_msgs REQUIRED)

# Header-Pfade: eigenes include + smov_lib (als Geschwisterordner in spot_master)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../smov_lib/include
)

add_executable(motion_cmd_node
  src/motion_cmd_node.cpp
  src/teleop_keys.cpp
)
ament_target_dependencies(motion_cmd_node rclcpp smov_lib smov_msgs)

install(TARGETS motion_cmd_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
