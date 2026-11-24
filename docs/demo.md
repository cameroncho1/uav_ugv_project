# UAV–UGV demo video

[Documentation](README.md) | [Validation and results](demo_validation.md) | [Recorded data](demo_results.json)

- Show a drone following a ground robot, camera mapping, A* routing, cooperative localization, fault injection, and learned sensor checks.
- Compare two estimates of the **same ground robot**. Blue uses UAV corrections. Orange uses GPS, wheels and IMU. The cooperative estimate controls the robot; orange is a display comparison. Simulator truth supplies the white vehicle and scoring reference.
- Record 3–5 minutes in chronological order. Cut idle startup and waiting. Keep each control change and its consequence together.

## Run

- Build in Linux using the [goal demo instructions](../README_simple_goal_demo.md#build). The [WSL setup](demo_validation.md#wsl-reproduction) uses Ubuntu-22.04, ROS 2 Humble, Gazebo Harmonic, PX4 SITL and MicroXRCEAgent. Windows provides editing and Git.
- Run from the built repository root:

  ```bash
  ./scripts/run_demo.sh
  ```

- Open <http://localhost:8765> beside Gazebo. The launcher starts the course, timed obstacle, detector, comparison estimator and dashboard. RViz is off. Extra launch arguments override defaults, including `start_rviz:=true` and `dashboard_port:=8800`.
- Frame the drone and ground robot in Gazebo. Record at 1440×960 or larger. Use a close Gazebo shot for takeoff, then keep the dashboard map and error panel readable.
- Start the driving segment at **Navigating**. Press Ctrl+C when finished and stop remaining processes belonging to that launch.

## Visual key

- **White vehicle and thick white track:** actual ground-robot position. **Blue:** cooperative estimate, drawn over white when they agree. **Dashed orange:** estimate without UAV corrections.
- **Green G:** goal. **S:** start. **Return goal** selects (-10, -3), left of S. Clicking clear ground selects another goal.
- **Lighter ground:** camera-seen ground. **Gray cells:** camera-detected obstacles. **Purple halo:** planner clearance near detections. Unseen ground and map edges also constrain planning.
- **Red outlines labeled Live block or Timed block:** simulator placement. They appear when a box is spawned. Gray occupied cells appear when the camera observes it. The planned route changes from mapped inputs.
- **Gold dots:** UGV GPS fixes. Increasing noise spreads the fixes; switching UGV GPS off stops them. The drone retains its own PX4 localization.
- **White diamond:** UAV position. **Blue circle:** twice the largest horizontal standard deviation. It displays reported uncertainty, not a measured confidence guarantee.
- **Action caption and chart markers:** acknowledged control changes. The chart shows the current 60-second window. Explain the current response while it is visible.
- **UGV GPS check:** combines GPS model flags with a missing-fix timeout. **Learned wheel/gyro flags:** BiLSTM predictions. These flags are displayed; the navigation position, map, route and command guards remain active. `fault_hold:=true` also connects detector decisions to supervisor holds.
- **UAV corrections:** switches estimator corrections while the drone keeps flying and mapping. UAV loss is outside the detector's monitored channels.

## Course

- Start (-4, 0), goal (9, 3): 13 m eastward displacement. A U-shaped pocket, a wall with a 2.0 m opening, cylinders and scattered blocks force detours.
- Press **Add block** 10 seconds into navigation. The red **Live block** label identifies the clicked box. The button reports rejected placements when passing clearance is unavailable.
- The **Timed block** appears at (7.1, 1.0), 22 seconds after the first drive command. Its red outline marks placement; gray camera cells supply the mapping evidence.
- `scripts/generate_obstacle_worlds.py --layout complex` generates the world. The UAV spawn has 0.8 m obstacle clearance. PX4 preflight checks stay enabled.

## Presentation script

- Use the video times to pace the edit. Use visible events to advance the explanation. Each row continues the same run.

| Video time | Screen and action | Narration |
| --- | --- | --- |
| 0:00–0:20 | Gazebo close shot: drone above the ground robot. Switch to the dashboard. | “The drone follows the ground robot with a downward camera. The camera builds the obstacle map, and the ground robot follows an A* route.” |
| 0:20–0:35 | Point to white, blue, orange, gray cells and gold dots. | “White is the actual vehicle. Blue uses UAV corrections. Orange estimates that same vehicle using GPS, wheels and IMU. Gray cells come from the camera. Gold dots are GPS fixes.” |
| 0:35–1:15 | Follow the pocket detour. Add the live block 10 seconds into navigation. Show the timed block and its camera detection. | “The route bends around the pocket. This click places the box labeled Live block. The second box is labeled Timed block. Red shows simulator placement; gray cells show what the camera detects. The planner routes around the mapped obstacles.” |
| 1:15–1:30 | Hold on **Goal reached** and the two error readings. | “The robot reaches its goal. These numbers compare both estimates against simulator truth. Blue lies inside the white track because the cooperative position is close to the actual path.” |
| 1:30–1:50 | Press **Return goal**. Set noise to 3× for 12 s, then 1× for 3 s. | “The new green goal redraws the route past the start area. Sensor noise is tripled. The GPS fixes spread, and the error chart shows each estimator's response. Noise returns to its nominal setting.” |
| 1:50–2:10 | Switch **UGV GPS** off. Observe for 8 s. | “The ground robot's GPS is off. The drone keeps its own localization. Gold fixes stop, and the GPS timeout reports missing data. Orange estimates motion from wheels and IMU; its short-term estimate is steadier without the noisy fixes.” |
| 2:10–2:40 | Switch **UAV corrections** off for 8 s, then on for 8 s. Keep the uncertainty circle, error panel and hold reason visible. | “UAV corrections are off while the camera keeps mapping. The blue uncertainty circle grows, and the position guard stops navigation. Corrections are on again. The circle contracts and the robot continues.” |
| 2:40–3:15 | Set **Wheel speed error** to +20% for 6 s. Add **Gyro drift** at +2°/s. Observe for 30 s. | “Measured wheel speed is twenty percent high. The gyro adds two degrees of heading bias per second. The learned wheel and gyro flags stay clear under this injection. The position guard stops navigation as reported uncertainty grows.” |
| 3:15–3:35 | Read the displayed hold reason. Press **Restore sensors** and observe for 8 s. End on the dashboard and vehicle. | “The robot is held because its reported position uncertainty exceeds the navigation limit. Restore sensors removes the live block and returns the injection settings to nominal. Integrated bias and filter state are preserved. The system combines camera mapping, route planning, cooperative localization, fault injection and learned sensor checks.” |

## Repeat the sequence

- Start a fresh demo, then run the rehearsal driver in a second Linux terminal:

  ```bash
  python3 scripts/rehearse_demo.py --output localization_results/demo_take
  ```

- The driver performs the presentation controls in order and saves actions, phase statistics, raw samples, navigation reasons and actual detector flags. The output directory must be new.
- **Restore sensors** changes settings and removes dashboard-added blocks. The timed box, integrated sensor bias and filter state stay in the run. Restart the launch to initialize a new take.
- The [validation report](demo_validation.md) states the measured conditions and results for this sequence.
