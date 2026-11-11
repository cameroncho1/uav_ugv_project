cmake_minimum_required(VERSION 3.8)
project(ugv_control)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_ugv_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SUPERVISOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(SUPERVISOR_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/ugv_control/supervisor_policy.hpp")
add_custom_command(OUTPUT "${SUPERVISOR_HEADER}"
  COMMAND "${Python3_EXECUTABLE}" "${SUPERVISOR_ROOT}/scripts/generate_supervisor_policy.py"
    --policy "${SUPERVISOR_ROOT}/verification/supervisor/policy.json"
    --output "${SUPERVISOR_HEADER}"
  DEPENDS "${SUPERVISOR_ROOT}/scripts/generate_supervisor_policy.py"
    "${SUPERVISOR_ROOT}/verification/supervisor/policy.json"
    include/ugv_control/supervisor.hpp include/ugv_control/supervisor_inputs.hpp
    src/supervisor_transition_probe.cpp src/goal_controller.cpp
  VERBATIM)
add_custom_target(supervisor_policy DEPENDS "${SUPERVISOR_HEADER}")
pkg_check_modules(GZ_SIM REQUIRED IMPORTED_TARGET gz-sim8)
pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs10)
pkg_check_modules(GZ_PLUGIN REQUIRED IMPORTED_TARGET gz-plugin2)
add_library(ugv_command_watchdog SHARED src/command_watchdog.cpp)
target_compile_features(ugv_command_watchdog PRIVATE cxx_std_17)
target_link_libraries(ugv_command_watchdog PRIVATE PkgConfig::GZ_SIM
  PkgConfig::GZ_TRANSPORT PkgConfig::GZ_MSGS PkgConfig::GZ_PLUGIN Threads::Threads)
target_include_directories(ugv_command_watchdog PRIVATE ${uav_ugv_msgs_INCLUDE_DIRS})
install(TARGETS ugv_command_watchdog LIBRARY DESTINATION lib)

add_executable(goal_controller src/goal_controller.cpp)
ament_target_dependencies(goal_controller rclcpp rcl_interfaces geometry_msgs nav_msgs std_msgs uav_ugv_msgs)
target_include_directories(goal_controller PRIVATE include "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_dependencies(goal_controller supervisor_policy)
target_compile_features(goal_controller PRIVATE cxx_std_17)

add_executable(supervisor_transition_probe src/supervisor_transition_probe.cpp)
target_include_directories(supervisor_transition_probe PRIVATE include
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_features(supervisor_transition_probe PRIVATE cxx_std_17)
add_dependencies(supervisor_transition_probe supervisor_policy)

install(TARGETS goal_controller supervisor_transition_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES "${SUPERVISOR_HEADER}" DESTINATION include/ugv_control)
ament_export_include_directories(include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_follower test/test_path_follower.cpp)
  target_include_directories(test_path_follower PRIVATE include)
  target_compile_features(test_path_follower PRIVATE cxx_std_17)
  ament_target_dependencies(test_path_follower geometry_msgs nav_msgs uav_ugv_msgs)
  ament_add_gtest(test_supervisor test/test_supervisor.cpp)
  target_include_directories(test_supervisor PRIVATE include "${CMAKE_CURRENT_BINARY_DIR}/generated")
  target_compile_features(test_supervisor PRIVATE cxx_std_17)
  add_dependencies(test_supervisor supervisor_policy)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(supervisor_conformance test/test_supervisor_conformance.py
    TIMEOUT 120
    ENV "SUPERVISOR_PROBE=$<TARGET_FILE:supervisor_transition_probe>"
    APPEND_ENV "PYTHONPATH=${SUPERVISOR_ROOT}/verification/src")
endif()
ament_package()
