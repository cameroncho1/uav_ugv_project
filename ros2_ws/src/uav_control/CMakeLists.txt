cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(offboard_goal_controller src/offboard_goal_controller.cpp)
ament_target_dependencies(offboard_goal_controller rclcpp px4_msgs geometry_msgs)

install(
  TARGETS offboard_goal_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()