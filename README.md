# UAV / UGV Project

Research and development project using ROS 2, PX4, and simulation.

The project architecture is intentionally kept modular and minimal at this
stage. ROS 2 packages and system components will be added as project
requirements become clearer.

## Repository Structure

    external/
        External dependencies such as PX4.

    ros2_ws/src/
        ROS 2 packages and interfaces.

    config/
        Project-wide configuration files.

    docs/
        Design notes and documentation.

    scripts/
        Setup, build, launch, and utility scripts.

    simulation/
        Simulation worlds, models, sensors, and configuration.

    tests/
        Unit and integration tests.

## Pinned Dependencies

PX4-Autopilot:

    d6f12ad1c4f70ad3230afd7d86e971421e02fef4

px4_msgs:

    86d8239e962f6939e05c3737784f60c02fa884db

Both are managed as Git submodules.

## Clone

    git clone --recurse-submodules \
      https://github.com/cameroncho1/uav_ugv_project.git

If already cloned without submodules:

    git submodule update --init --recursive
