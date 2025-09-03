cmake_minimum_required(VERSION 3.16)
project(motion_cmd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_lib REQUIRED)
find_package(smov_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../smov_lib/include
)

# Gait-Node (wie zuvor)
add_executable(motion_cmd_node
  src/motion_cmd_node.cpp
  src/teleop_keys.cpp
)
ament_target_dependencies(motion_cmd_node rclcpp smov_lib smov_msgs)
target_link_libraries(motion_cmd_node smov_lib)   # ggf. zu smov_lib::smov_lib ändern

# NEU: Tastatur-Teleop, publisht /cmd_vel, /walk_cmd, /idle_cmd
add_executable(teleop_kb
  src/teleop_kb.cpp
  src/teleop_keys.cpp
)
ament_target_dependencies(teleop_kb rclcpp geometry_msgs std_msgs)

install(TARGETS motion_cmd_node teleop_kb
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
