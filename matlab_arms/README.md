# E-Waste Battery Extraction — MATLAB Physics

MATLAB code for forward/inverse kinematics, trajectory planning, and joint torque validation for the two 7-DoF arms I considered for the project:

- Kinova Gen3 (primary simulation platform)
- UFACTORY xArm 7 (physical deployment platform at the Burwood Robotics Lab)

Everything here is simulation/model-based. I did not have physical hardware to test on this trimester that's the T2 deliverable.

## Layout

```
kinova_gen3/      Gen3 model, FK, IK, torques
xarm7/            xArm 7 model, FK, IK, torques
common/           trajectory_generate, validate_trajectory
scripts/          runnable demos for each arm
urdf/             Vendored URDFs (Gen3 7-DoF + xArm 7)
```

## Requirements

- MATLAB R2020a or newer
- Robotics System Toolbox

## How to run

From the MATLAB prompt:

```matlab
run scripts/demo_kinova_gen3_battery_extraction.m
run scripts/demo_xarm7_battery_extraction.m
```

Each demo plans a 5-waypoint trajectory (home → above phone → grasp → lift → deposit), computes joint torques with a 50 g payload, and checks against the manufacturer's joint, velocity, and torque limits. Plots and a CSV summary are saved into `results/<arm>/`.

A PASS verdict means every joint stays below 95% of its published limit. WARNING means a joint is approaching the limit (and the waypoint timing probably needs adjusting). FAIL means an over-limit condition.

## Joint torque limits

| Arm   | J1  | J2  | J3  | J4  | J5  | J6  | J7  |
| ----- | --- | --- | --- | --- | --- | --- | --- |
| Gen3  | 39  | 39  | 39  | 39  | 9   | 9   | 9   |
| xArm7 | 50  | 50  | 30  | 30  | 30  | 20  | 20  |

All values in Nm, from the manufacturer URDFs.

## URDF sources

- **Kinova Gen3**: `gen3_7dof.urdf` is the official URDF from the `ros_kortex` repository (`kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf`).
- **xArm 7**: `xarm7.urdf` is the xacro-generated URDF from `xarm_description`, shared by Vinh in the team channel.

Mesh `.STL` files are not included — kinematics and dynamics work without them, you just don't get visual meshes in `show(robot)`. If you want the meshes, clone the upstream repos (`Kinovarobotics/ros_kortex` and `xArm-Developer/xarm_ros2`).

## Maintainer

- **Tarunjeet Singh** Team Lead, Robotics Sub-Team (Sprint 1, 2)