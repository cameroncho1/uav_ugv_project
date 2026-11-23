# Presentation demo (3–5 minutes)

[Documentation index](README.md) | [Navigation](navigation.md) | [Dynamic obstacles](dynamic_obstacles/README.md) | [Validation](demo_validation.md)

The demo shows camera mapping, A* routing through a 12-obstacle course, two obstacles added during navigation, cooperative localization, and the learned sensor-fault detector. The dashboard compares two estimators on the **same physical vehicle and sensor stream**. The orange estimator never receives UAV corrections; it does not control a second vehicle.

## Run it

Build in Linux as in the [goal demo](../README_simple_goal_demo.md#build), then run from the repository root:

```bash
./scripts/run_demo.sh
```

Open <http://localhost:8765> beside Gazebo. Extra arguments pass to the launch file, including `start_rviz:=false` and `dashboard_port:=8800`. Ctrl+C stops the launch. Windows is for editing and Git; [WSL setup and bounded execution](demo_validation.md#wsl-reproduction) use Ubuntu-22.04, ROS 2 Humble, Gazebo Harmonic and MicroXRCEAgent. Node is unnecessary.

Wait for **Live simulation**, then **Navigating**. In the recorded run, the UGV starts about 22 simulation seconds after truth becomes available. If it remains waiting for the UAV, inspect PX4's preflight output and the spawn area; do not disable preflight checks. The generated complex course reserves 0.8 m around the UAV spawn, including rejecting obstacles that cover it.

The detector displays predictions. It can raise false flags. `fault_hold:=true` additionally connects its decisions to supervisor holds; the presentation defaults to display only, as the dashboard states. The independent position, map, route and command guards remain active.

## Read and operate the dashboard

- The white vehicle and path show simulator truth. Blue shows the cooperative estimate; dashed orange shows GPS/wheel/IMU only. Their position errors share the same colors and a 60-second chart.
- Errors and trails use interpolated acquisition times with a short display delay. Stale inputs leave gaps. The blue circle has radius twice the largest horizontal standard deviation; it is a conservative display circle, not a measured confidence guarantee.
- Lighter ground is camera-seen ground. Gray cells are detected obstacles; the surrounding halo is clearance near those detections. Faint outlines show static world geometry for the audience. Unseen-ground and map-edge padding also constrain the planner but are omitted from this display halo.
- The green bullseye is the current goal. Click clear ground to change both goal coordinates atomically. The dashboard reports rejection reasons, including goals near active hazard boxes. The dashboard and RViz show live goals; the Gazebo bullseye is a best-effort visual update.
- **Add block** places a 0.6 m box ahead on the route when there is room to pass. A refused placement explains why; the timed hazard still demonstrates an obstacle appearing during the run.
- **UAV corrections** gates cooperative measurements. The UAV keeps flying and mapping. **Wheel speed error** scales measured forward speed; **Gyro drift** adds a growing heading bias. These are injected simulation assumptions.
- **GPS noise (1σ)** is the unscaled horizontal standard deviation. **Sensor noise** multiplies it and the other configured sensor standard deviations.
- **Restore sensors** restores nominal settings and removes dashboard-added blocks. It preserves the tracks and error history. It does not rewind the world, remove the timed hazard, reset either filter, or erase integrated gyro bias. Restart the launch for a fresh presentation.
- A disconnected server or paused simulation clock is stated explicitly; a frozen page is not evidence of continued navigation.

## The course

- Start (-4, 0), goal (9, 3): 13 m eastward displacement, with a longer driven detour.
- A U-shaped pocket opens toward the start. The route goes around it as the camera maps it.
- A wall at x = 3 reaches the usable map edges, with a 2.0 m opening at y in [-2, 0].
- Cylinders and scattered blocks follow the opening.
- A timed 0.6 m box appears at (7.1, 1.0), 22 seconds after the first drive command, through `complex_course_box.yaml`.
- `scripts/generate_obstacle_worlds.py --layout complex` generates the world. Dark, low obstacles suit the existing luminance mapper and flat-ground projection.

## Presentation script

Times are approximate and start with the first drive command. Follow the visible events if the simulation runs slowly. The complete recorded run takes about 3.6 simulation minutes after truth starts, including startup and recovery observation.

| Time | Do | Say |
| --- | --- | --- |
| Before | Launch; show Gazebo and the dashboard. | “The UAV follows a ground vehicle with a downward camera. This map comes from those images.” |
| 0:00 | Point to the lighter ground, route and pocket. | “The planner can search unseen ground, but the vehicle drives only the section confirmed clear. As the pocket appears in the map, the route bends around it.” |
| 0:10–0:20 | Press **Add block** once in open space. If refused, keep driving. | “This box enters through the simulator. The planner learns about it through the camera.” |
| 0:22 | Point to the timed red box. | “A second obstacle appears on a timer. Watch the detected cells and the remaining route as the camera reaches it.” |
| About 1:10 | Wait for **Goal reached**; point to the position errors. | “White is the actual path. Blue uses UAV corrections; orange uses GPS, wheels and IMU only. Both estimates receive the same ground-vehicle measurements.” |
| 1:15 | Click clear ground near (-4, -3). Set **Sensor noise** to 3× for about 15 seconds. | “The goal changes during the run. Now the sensor noise is tripled. The chart shows how each estimate responds.” |
| 1:35 | Return noise to 1×; switch **GPS** off. | “GPS is unavailable. The UAV still supplies independent position observations. Detector flags are predictions; they are displayed separately from injected settings.” |
| 1:55 | Set **Wheel speed error** to +20% and **Gyro drift** to +2°/s. | “Now speed and heading measurements are corrupted too. Compare the error growth and watch the navigation state. A position-uncertainty hold is an expected limit under these combined faults.” |
| 2:15 | Switch **UAV corrections** off for 20 seconds, then on. | “This removes corrections from the estimator while the camera keeps mapping. Turning them back on does not guarantee recovery after large drift.” |
| About 3:05 | Press **Restore sensors**; show the retained chart and hold reason if present. | “The settings return to nominal, and the live block is removed. The accumulated filter and sensor state remains visible. This run demonstrates the benefit of assistance and the point where the navigation guard refuses to proceed.” |

Do not attribute a hold to the last switch pressed without checking its reason: in the recorded run, uncertainty triggers a hold **before** UAV corrections are disabled. Do not promise that blue stays fixed under noise, that the detector always responds within one second, or that restoring settings instantly recovers localization.

## Recorded conditions

The [retained review result](demo_validation.md) records one seed on flat simulated ground. With both added obstacles present, the first goal is reached about 68 seconds after the first observed navigation state, 0.374 m from the goal in simulator truth. During that course, mean position error is 0.031 m with UAV corrections and 0.354 m without them. Under GPS loss plus injected wheel/gyro faults, both error and uncertainty grow and the supervisor holds.

These are measurements of this configuration, not hardware accuracy or general fault-tolerance guarantees.
