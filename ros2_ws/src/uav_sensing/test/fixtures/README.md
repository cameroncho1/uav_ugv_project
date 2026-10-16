# Rendered detector fixture

`off_axis_tag.png` is a 1280×960 image from `fiducial_camera_test.sdf`, with the target moved to world `(1, -0.5, 0.37)` m and the observer body at `(0, 0, 4)` m. Gazebo 8.15.0 uses the Ogre server renderer. The target is AprilTag 36h11 ID 7, 0.4 m black edge.

OpenCV 4.5.4 misses this frame with adaptive threshold constant 7 and detects ID 7 with constant 15. The unit check requires detection and rejects the same image with half the code obscured. The live fixture separately checks metric position, occlusion, and recovery.
