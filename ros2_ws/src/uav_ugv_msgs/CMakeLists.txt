cmake_minimum_required(VERSION 3.8)
project(uav_ugv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/PlannedRoute.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(geometry_msgs nav_msgs rosidl_default_runtime)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pose_guard_test test/test_pose_guard.cpp)
  target_include_directories(pose_guard_test PRIVATE include)
  target_compile_features(pose_guard_test PRIVATE cxx_std_17)
  ament_target_dependencies(pose_guard_test geometry_msgs)
  ament_add_gtest(route_safety_test test/test_route_safety.cpp)
  target_include_directories(route_safety_test PRIVATE include)
  target_compile_features(route_safety_test PRIVATE cxx_std_17)
  ament_target_dependencies(route_safety_test geometry_msgs nav_msgs)
  rosidl_get_typesupport_target(route_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(route_safety_test "${route_typesupport}")
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
