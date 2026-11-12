cmake_minimum_required(VERSION 3.8)
project(fault_management)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/sim_ugv_sensors.py scripts/sim_uav_sensor.py
  scripts/sim_anchor_fault.py scripts/localization_trial.py
  scripts/ugv_dataset_recorder.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/hazard_observer.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/simulation_faults.py scripts/hazard_assessment.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(localization_trial_test test/test_localization_trial.py)
  ament_add_pytest_test(sim_sensor_test test/test_sim_sensors.py)
  ament_add_pytest_test(launch_modes_test test/test_launch_modes.py)
  ament_add_pytest_test(ugv_dataset_recorder_test test/test_ugv_dataset_recorder.py)
  ament_add_pytest_test(hazard_assessment_test test/test_hazard_assessment.py)
  ament_add_pytest_test(verification_test ../../../verification/tests
    TIMEOUT 300
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../../verification/src)
  ament_add_pytest_test(verification_bag_test test/test_verification_bag.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../../verification/src)
  ament_add_pytest_test(experiment_process_test test/test_experiment_processes.py
    TIMEOUT 60
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../../verification/src)
endif()
ament_package()
