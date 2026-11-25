# UAV–UGV demo video

[Documentation](README.md) | [Validation](demo_validation.md) | [Navigation results](demo_results.json) | [ML results](ml_demo_results.json)

- Record one 3–5 minute course-project video with two chapters: **camera-assisted navigation and localization**, then **live ML fault detection and recovery**. Each chapter uses a fresh simulation and proceeds in chronological order. Cut startup and idle waiting; keep each action with its response.
- The drone follows the ground robot, its downward camera builds the obstacle map, and A* plans the ground robot's route. Cooperative localization controls the robot. A second estimator uses GPS, wheels and IMU for comparison. Simulator truth supplies display and scoring data.
- The ML chapter runs the exported paired time-domain BiLSTM on live sensor windows. The screen connects measured input, independent reference, model score, persistence, published fault and supervisor response.

## Run and record

- For a scripted silent video while continuing to use the Windows desktop, use the [private-display recording workflow](demo_recording.md). It runs Gazebo on an isolated virtual Linux display, captures the dashboard headlessly, and assembles both chronological chapters with explicit pacing and evidence.

- Build in Linux using the [goal demo instructions](../README_simple_goal_demo.md#build) or [WSL reproduction](demo_validation.md#wsl-reproduction). Runtime dependencies are Ubuntu-22.04, ROS 2 Humble, Gazebo Harmonic, PX4 SITL and MicroXRCEAgent. Windows provides editing and Git. The dashboard needs no Node runtime.
- Start chapter 1 from the built repository root:

  ```bash
  ./scripts/run_demo.sh
  ```

- Open <http://localhost:8765> beside Gazebo. The launcher selects **Navigation**, the 13 m course, the timed obstacle and display-only detector decisions. RViz is off. Extra launch arguments override defaults, including `start_rviz:=true` and `dashboard_port:=8800`.
- Gazebo uses the [project viewer configuration](../simulation/gui/demo.config), with a course overview and no transform editor. To select a different layout, run `GZ_GUI_CONFIG=/path/to/gui.config ./scripts/run_demo.sh`.
- Record at 1440×960 or larger. Frame the drone and ground robot in Gazebo for takeoff; keep the dashboard map and error panel readable during driving. Begin controls at **Navigating**.
- Run the presentation controls automatically in a second Linux terminal:

  ```bash
  python3 scripts/rehearse_demo.py --output localization_results/navigation_take
  ```

- Stop the launch with Ctrl+C and stop its remaining simulation processes. Start a fresh simulation for chapter 2:

  ```bash
  ./scripts/run_ml_demo.sh
  ```

- Reload the dashboard. The launcher selects **ML detector**, an unobstructed straight drive from (-4, 0) to (4, 0), and detector-driven holds. Wheel speed error starts at 0%. Keep **Wheel forward speed (m/s)** selected.
- Run the ML controls in a second Linux terminal:

  ```bash
  python3 scripts/rehearse_ml_demo.py --output localization_results/ml_take
  ```

- Each output directory must be new. Both drivers save `summary.json` and raw `samples.jsonl`. The ML driver finishes at goal arrival with sensors nominal and UAV measurements enabled. Hold the final shot on **Goal reached**, the clear wheel flag and the stopped robot. Stop the owned simulation processes when recording finishes.
- Optional UAV timeout validation uses `python3 scripts/rehearse_ml_demo.py --check-uav-timeout --output localization_results/ml_timeout_check` against a fresh ML run. This adds outage and restoration checks after goal arrival. Keep this sequence in separate validation footage.

## Navigation screen and course

- **White vehicle and thick white track:** actual ground-robot position. **Blue:** cooperative estimate, drawn over white when they agree. **Dashed orange:** estimate of the same robot without UAV corrections.
- **Green G:** goal. **S:** start. **Return goal** selects (-10, -3), left of S. Clicking clear ground selects another goal.
- **Lighter ground:** camera-seen ground. **Gray cells:** camera-detected obstacles. **Purple halo:** planner clearance near detections. Unseen ground and map edges also constrain planning.
- **Red outlines labeled Live block or Timed block:** simulator placement. Gray occupied cells appear when the camera observes the box. The planner consumes the camera map.
- **Gold dots:** UGV GPS fixes. Noise spreads the fixes; switching UGV GPS off stops them. The drone retains its own PX4 localization.
- **White diamond:** UAV position. **Blue circle:** twice the largest reported horizontal standard deviation. It displays filter uncertainty. Position errors compare estimates with truth at matching acquisition times.
- **Action caption and chart markers:** acknowledged control changes. The error chart shows 60 seconds. Explain each response while it is visible.
- Start (-4, 0), goal (9, 3): 13 m eastward displacement. A U-shaped pocket, a wall with a 2.0 m opening, cylinders and scattered blocks force detours. **Add block** places the live box 10 seconds into navigation. The timed box appears at (7.1, 1.0), 22 seconds after the first drive command.
- `scripts/generate_obstacle_worlds.py --layout complex` generates the course. The UAV spawn has 0.8 m obstacle clearance. PX4 preflight checks stay enabled.

## ML screen and decisions

- **Measured sensor / Command or wheel reference:** raw values supplied to the paired model, sampled at each inference window's last acquisition time. Wheel forward speed uses the drive command as its reference; gyro turn rate uses wheel odometry. The plot displays those window endpoints over 30 seconds.
- **BiLSTM fault score:** live classifier output. A score above 0.50 flags the raw window. The score is a model output, not calibrated confidence.
- **Consecutive flagged decision windows:** the detector's actual persistence count. Three distinct consecutive flagged decisions raise the wheel or gyro class flag. **Published detector flag** shows the resulting decision. The dashboard displays telemetry; inference and persistence execute in the detector.
- Each input window contains 100 measured samples. Inference runs every 0.5 simulation seconds over overlapping windows. The exported weights, standardization, threshold 0.50 and three-decision persistence remain fixed. The autoencoder is disabled by default.
- **Holding** with **Fault hold** identifies the navigation supervisor's response to the published detector flag. **Restore sensors** returns controls to nominal; it preserves integrated bias, filter state and trace history. Clean model decisions and the supervisor's clear interval permit navigation to resume.
- **UAV corrections** controls whether the cooperative estimator uses UAV observations. **UAV measurement stream** controls the independent relative-observation gate. The drone keeps flying and mapping. The separate **UAV measurement timeout** status reports missing observations after two seconds; it is a timeout rule.
- **UGV GPS check** combines learned GPS outputs with a missing-fix timeout. The demonstrated learned stop uses the wheel-speed class. The gyro selector exposes the live gyro channel without claiming a demonstrated gyro stop.

## Presentation script

- Follow visible events within each chapter. The video times allocate room for narration and a clear chapter transition.

| Video time | Screen and action | Narration |
| --- | --- | --- |
| 0:00–0:20 | Gazebo close shot: drone follows the ground robot. Switch to Navigation. | “The drone follows the ground robot with a downward camera. The camera builds this obstacle map, and the ground robot follows an A* route.” |
| 0:20–0:40 | Point to white, blue, orange, gray cells and gold dots. | “White is the actual vehicle. Blue uses UAV corrections. Orange estimates that same vehicle using GPS, wheels and IMU. Gray cells are camera detections; gold dots are GPS fixes.” |
| 0:40–1:15 | Show the pocket detour. Add the live block 10 seconds into navigation; show the timed block and gray camera detections. | “The route bends around the pocket. This control places the Live block. The Timed block appears farther along the course. Red shows placement; gray shows camera detection. The route follows the mapped free space.” |
| 1:15–1:30 | Hold on Goal reached and both error readings. Press Return goal. | “The robot reaches its goal. Both position errors are measured against simulator truth. The return goal gives us a second route for the sensor controls.” |
| 1:30–1:50 | Set noise to 3× for 12 s, then 1× for 3 s. | “The sensor noise is tripled. GPS fixes spread, and both error traces show the response. Noise returns to its nominal setting.” |
| 1:50–2:10 | Switch UGV GPS off for 8 s. | “The ground robot's GPS is off. Gold fixes stop, and the missing-fix timeout reports the outage. The drone keeps its own localization and continues supplying observations.” |
| 2:10–2:40 | Switch UAV corrections off for 8 s, then on for 8 s. Restore sensors and observe for 8 s. | “UAV corrections are off while the camera keeps mapping. The blue uncertainty circle grows, and the position guard stops navigation. Corrections return, uncertainty contracts, and navigation resumes. Restore sensors returns the controls to nominal and removes the live block.” |
| 2:40–3:00 | Chapter card: Live ML fault detection. Fresh straight-drive simulation, ML detector view. Show clean input and score. | “This straight drive shows the learned sensor detector. The measured wheel speed tracks the command reference. The BiLSTM reads paired sensor windows and produces this fault score.” |
| 3:00–3:25 | Set Wheel speed error to +60%. Show input separation, score crossing 0.50, persistence 1/3, 2/3, 3/3 and FAULT. Keep Holding visible. | “Wheel speed is reported sixty percent high. The measured input separates from its reference. The learned score crosses the threshold. Three consecutive flagged decisions publish a wheel fault, and the supervisor holds the robot.” |
| 3:25–3:55 | Return Wheel speed error to 0%. Show the score clearing, navigation resuming and goal arrival. Keep Gazebo visible beside the ML readings. | “The injected error is removed. Clean model decisions clear the flag. The supervisor's clear interval completes, the robot resumes, and it reaches the goal.” |
| 3:55–4:10 | Hold on Goal reached, the clear wheel flag and the robot at its goal. End without another injection. | “The robot reaches its destination with the fault cleared. Camera mapping, cooperative localization, route planning and learned fault detection support the completed drive.” |

- The [validation report](demo_validation.md) links exact measured conditions, test results, screenshots and both chronological event traces.
