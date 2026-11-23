# Presentation demo (about 4 minutes)

[Documentation index](README.md) | [Navigation](navigation.md) | [Dynamic obstacles](dynamic_obstacles/README.md) | [Goal demo](../README_simple_goal_demo.md)

One run shows the system the team built: UAV-assisted localization against a GPS/wheel/IMU-only estimate, camera mapping and A* routing through a 12-obstacle course, obstacles that appear mid-run, and a live sensor-fault detector. A dashboard shows all of it and changes the sensors, noise, goal and obstacles while the vehicle drives.

## Run it

Build as in the [goal demo](../README_simple_goal_demo.md#build), then from the repository root in an Ubuntu desktop session:

```bash
./scripts/run_demo.sh
```

Open <http://localhost:8765> next to the Gazebo window. Extra arguments pass to the launch file, for example `start_rviz:=false` or `dashboard_port:=8800`. Ctrl+C stops every process. The vehicle starts driving about 20 s after the UAV takes off.

`fault_hold:=true` lets detector decisions hold the UGV. The learned detector raises brief false alarms in nominal runs (five holds in one 95 s run), so the demo leaves hold off and only displays detector flags.

## The dashboard

- **Map.** Ground the UAV camera has seen is lighter; unseen ground stays dark. Obstacles appear as the camera detects them, with the planner's clearance halo around them. Faint outlines show the world's true obstacles.
- **Tracks.** Truth (white), cooperative estimate (blue, with its 2-sigma circle), GPS/wheel/IMU-only estimate (orange), recent GPS fixes (red dots). The UAV is the hollow diamond.
- **Route.** The dashed teal line is the remaining route; short dots mean the route stops at the edge of seen ground. It glows when a new obstacle appears.
- **Click the map** to send the UGV to a new goal. The ring turns red over obstacles.
- **Add block** drops a 0.6 m box on the route about 2 m ahead. It refuses (button flashes) when no spot leaves room to pass. Reset removes added blocks.
- **Top right.** Navigation state and any active detector flags.
- **Errors.** Position error of the two estimates against truth, now and over the last 60 s.
- **Controls.** GPS and UAV switches; sliders for noise on every sensor, GPS error, wheel slip and gyro drift. Reset returns all to nominal.

## The course

- Start (-4, 0), default goal (9, 3), about 17 m of path at up to 0.5 m/s.
- A U-shaped pocket sits straight ahead of the UGV, opening toward it; the route goes around.
- A wall at x = 3 runs to the edge of the mapped area with one 2.0 m opening.
- A cylinder slalom and scattered blocks follow before the goal.
- A 0.6 m box appears 22 s after the first drive command, past the wall opening (`complex_course_box.yaml`).
- The world comes from `scripts/generate_obstacle_worlds.py --layout complex`.

## Script

Times are from the first drive command unless noted. Speak the quoted lines; the bullets are what to do.

| Time | Do | Say |
| --- | --- | --- |
| Before | Launch. The dashboard shows the UAV climbing. | "A UAV and a ground vehicle. The UAV follows the vehicle and watches the ground with a downward camera." |
| 0:00 | UGV starts. Point at the lighter ground growing. | "The vehicle plans through ground it hasn't seen and drives only the part the camera has confirmed clear." |
| 0:08 | The route bends around the U. | "The straight line leads into a dead end. As the camera sees it, the route goes around." |
| 0:12 | Press **Add block**. | "Now I drop a block on its path. The camera sees it and the route changes." |
| 0:22 | The scenario box flashes red past the wall opening. | "A second one appears on a timer. Nobody told the planner either of them were there." |
| 1:00 | UGV reaches the goal. Point at the two numbers. | "Blue uses the UAV, orange uses GPS, wheels and the gyro only. Same sensors, the UAV is the difference." |
| 1:15 | Click a new goal. Raise **Noise** to 3x. | "Triple the sensor noise. Orange gets noisier. Blue does not move." |
| 1:45 | Lower **Noise**. Turn **GPS** off. | "Cut GPS. The detector flags it within a second." |
| 2:15 | Drag **Wheel slip** to +20% and **Gyro drift** up. | "Now the wheels and the gyro lie too. Orange drifts toward two metres. Blue stays well under it." |
| 2:45 | Switch **UAV** off. | "Remove the UAV and the safety guard stops the vehicle once its position uncertainty passes the limit." |
| 3:15 | Click **Reset**. | "Every control returns to nominal and the added blocks go away." |

## Measured in this configuration

From scripted runs of the course in Gazebo Harmonic (simulation assumptions, one seed, positions against simulator truth):

- Goal reached 54 s after the first drive command with the scenario box present, and 64 s with one added block.
- Nominal, GPS on: cooperative error about 0.02 to 0.05 m; GPS/wheel/IMU-only 0.2 to 0.4 m.
- GPS switched off: the detector's GPS flag appeared within 1 s.
- Noise at 3x with GPS on: GPS/wheel/IMU-only error rose from 0.36 to 0.7 m; cooperative error did not change.
- Wheel slip +20% and gyro drift 2 deg/s with GPS on: negligible change in either estimate (GPS corrects it).
- The same faults with GPS off: GPS/wheel/IMU-only error averaged 1.1 m (max 1.9 m); cooperative error averaged 0.28 m (max 0.8 m).
- UAV assistance off during those faults: the pose guard held the vehicle. Switching assistance back on did not reduce the error within 30 s.

These values apply to the recorded conditions only: flat ground, simulated noise and faults, one seed.
