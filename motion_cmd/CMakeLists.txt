cmake_minimum_required(VERSION 3.16)
project(motion_cmd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(smov_msgs REQUIRED)

# smov_lib stellt Header bereit
# In deinem Workspace ist smov_lib bereits gebaut/installiert (include/smov/*.h)
# Falls die Includes nicht automatisch gefunden werden, kannst du hier noch:
# include_directories(${CMAKE_INSTALL_PREFIX}/include)

include_directories(
  include
)

add_executable(motion_cmd_node
  src/motion_cmd_node.cpp
  src/teleop_keys.cpp
)
ament_target_dependencies(motion_cmd_node
  rclcpp std_msgs geometry_msgs smov_msgs
)

install(TARGETS motion_cmd_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
