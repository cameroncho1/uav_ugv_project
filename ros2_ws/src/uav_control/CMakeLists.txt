cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(odometry_listener
  src/odometry_listener.cpp
)

ament_target_dependencies(
  odometry_listener
  rclcpp
  px4_msgs
)

install(
  TARGETS odometry_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()