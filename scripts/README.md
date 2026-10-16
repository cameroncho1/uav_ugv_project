# Scripts

Run these scripts inside Ubuntu.

For the complete camera-assisted navigation demo, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mission_manager simple_goal_demo.launch.py
```

| Group | Scripts | Purpose |
| --- | --- | --- |
| Basic startup | `start_sim.sh`, `start_sim_tmux.sh` | Start the basic simulation in terminal tabs or tmux. |
| Individual processes | `world.sh`, `px4.sh`, `dds.sh`, `gui.sh` | Start Gazebo, PX4, DDS, or the GUI separately. |
| Camera-demo startup | `world_goal_demo.sh`, `px4_goal_demo.sh` | Start the matching camera world and UAV. |
| Localization experiments | `run_localization_trials.sh`, `run_cooperative_trials.sh` | Test baseline localization or compare baseline and UAV-assisted filters. |
| Flying-UAV experiments | `run_live_cooperative_trial.sh`, `run_aligned_anchor_trials.sh`, `run_camera_anchor_trials.sh` | Run PX4 trials; matrix wrappers repeat seeds and GPS cases. |
| Camera checks | `run_camera_perception_trial.sh`, `test_camera_perception.py` | Launch and score visibility, occlusion, and recovery tests. |
| Navigation experiments | `run_goal_navigation_trial.sh`, `run_path_tracking_trials.py` | Measure goal arrival and baseline/assisted route tracking. |
| Scoring | `score_goal_navigation.py`, `score_px4_heading.py` | Calculate navigation and UAV heading errors. |
| Behavior checks | `test_navigation_guards.py`, `test_actuator_watchdog.py`, `test_gnss_innovation.py` | Check invalid-input stops, command expiry, and GPS rejection. |
| Helpers | `check_sim_dependencies.py`, `process_cleanup.sh`, `evaluation_math.py`, `generate_fiducial_model.py` | Check dependencies, stop trial processes, calculate metrics, and generate the roof tag. |

[Setup and demo instructions](../README_simple_goal_demo.md) · [Evaluation commands](../docs/cooperative_localization/interfaces.md#evaluation-and-reproducibility)
