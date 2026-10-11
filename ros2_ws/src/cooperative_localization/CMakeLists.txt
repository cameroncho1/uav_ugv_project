cmake_minimum_required(VERSION 3.8)
project(cooperative_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs10)

add_executable(ugv_truth src/ugv_truth.cpp)
ament_target_dependencies(ugv_truth rclcpp geometry_msgs)
target_link_libraries(ugv_truth PkgConfig::GZ_TRANSPORT PkgConfig::GZ_MSGS)
install(TARGETS ugv_truth DESTINATION lib/${PROJECT_NAME})

add_executable(sim_vehicle_poses src/sim_vehicle_poses.cpp)
ament_target_dependencies(sim_vehicle_poses rclcpp geometry_msgs)
target_link_libraries(sim_vehicle_poses PkgConfig::GZ_TRANSPORT PkgConfig::GZ_MSGS)
install(TARGETS sim_vehicle_poses DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ugv_measurements.py scripts/ugv_fused_pose.py scripts/localization_trial.py
  scripts/uav_pose.py scripts/uav_correction.py scripts/sim_uav_sensor.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/cooperative_math.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ugv_measurements_test test/test_ugv_measurements.py)
  ament_add_pytest_test(ugv_fused_pose_test test/test_ugv_fused_pose.py)
  ament_add_pytest_test(localization_trial_test test/test_localization_trial.py)
  ament_add_pytest_test(uav_correction_test test/test_uav_correction.py)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
