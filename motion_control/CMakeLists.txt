 cmake_minimum_required(VERSION 3.8)
project(motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_lib REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  smov_lib
)

add_library(motion_control_lib SHARED src/motion_control.cc)
ament_target_dependencies(motion_control_lib ${dependencies})

add_executable(state src/motion_control.cc)
target_link_libraries(state motion_control_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state motion_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(motion_control_lib)
ament_export_dependencies(${dependencies})

ament_package()
