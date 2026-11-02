cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_ugv_msgs REQUIRED)

add_executable(offboard_goal_controller src/offboard_goal_controller.cpp)
ament_target_dependencies(offboard_goal_controller rclcpp px4_msgs geometry_msgs uav_ugv_msgs)
target_include_directories(offboard_goal_controller PRIVATE include)
target_compile_features(offboard_goal_controller PRIVATE cxx_std_17)

install(
  TARGETS offboard_goal_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/uav_pose.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(uav_pose_test test/test_uav_pose.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_follow_setpoint test/test_follow_setpoint.cpp)
  target_include_directories(test_follow_setpoint PRIVATE include)
  target_compile_features(test_follow_setpoint PRIVATE cxx_std_17)
endif()
ament_package()