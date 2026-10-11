# Repository instructions

## Runtime and validation

- Run ROS 2 Humble, PX4 SITL, and Gazebo Harmonic in Linux. Windows is an editing and Git environment.
- Build the packages touched by a change and run their registered `colcon test` checks.
- For localization changes, run the paired trials in `docs/cooperative_localization/interfaces.md` when the measurement or filter behavior changes.
- Stop simulation processes after evaluation. Keep unrelated working-tree changes intact.

## Localization boundaries

- Use ENU `map` for shared global positions and FLU for ROS body frames. Convert PX4 NED/FRD explicitly, including covariance and the local origin.
- Preserve acquisition timestamps. Simulation consumers use `/clock`; PX4 anchors require matching sample time.
- Keep simulator truth in sensor synthesis and scoring. Estimator and navigation code must consume stated measurement interfaces.
- Include UAV localization uncertainty in cooperative measurements. Do not feed the fused UGV estimate back into its UAV measurement producer.
- Keep numerical frame and covariance code independent of ROS callbacks and Gazebo transport.
- Treat injected noise and fault values as simulation assumptions. Limit claims to the recorded test conditions.

## Documentation and Git

- Describe implemented behavior and measured conditions in present tense. Link results, assumptions, and runnable commands.
- Keep `AGENTS.md` as shared instructions and `CLAUDE.md` as its import.
- Commit subjects must be concise; bodies contain 1–4 `-` bullets summarizing the change.
