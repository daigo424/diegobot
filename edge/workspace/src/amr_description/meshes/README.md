# Meshes

Some meshes in this directory are taken or derived from third-party projects.
Files not listed below are original to this repository and licensed under the MIT License (see the repository root `LICENSE`).

## Third-party meshes

| File | Source | License | Changes |
|---|---|---|---|
| `arm/*` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (SO-101) | Apache-2.0 | None |
| `arm_camera_mount.stl` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (SO-101) | Apache-2.0 | Modified from an SO-101 part |
| `chassis_base.stl` | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) (TurtleBot3 Waffle) | Apache-2.0 | Some parts removed, renamed |
| `left_tire.stl`, `right_tire.stl` | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) (TurtleBot3 Waffle) | Apache-2.0 | Renamed |

The URDF files that use these meshes are derived from the same projects:
`../urdf/so101_new_calib.urdf.xacro` from SO-ARM100 and `../urdf/chassis_base.urdf.xacro` from the TurtleBot3 Waffle URDF.

A copy of the Apache License 2.0 is in `../LICENSE-Apache-2.0`.

## Meshes not included

The following meshes are referenced by the URDF but not distributed here because no redistribution terms are given for them.
Without them, only the sensors' visuals are missing in RViz and Gazebo; the sensors and TF still work.

### Nuwa-HP60C camera

Download the 3D model from the "3D_Model_File" link on [Yahboom's HP60C page](https://www.yahboom.net/study/hp60c) and save it in this directory as `camera_NUWA_HP60C.stl`.

### YDLIDAR T-mini Plus

Download the [T-Mini Plus LiDAR model by KaiserGabo on Thangs](https://thangs.com/designer/KaiserGabo/3d-model/T-Mini%20Plus%20LiDAR-1309417) and save it in this directory as `lidar.stl`.
Before saving, move it so that the origin is at the center of the bottom face with +Z up (units: mm), which is what `../urdf/t_mini_plus_12.urdf.xacro` expects.
