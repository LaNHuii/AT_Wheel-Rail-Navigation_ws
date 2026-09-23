# AT Wheel-Rail Navigation Workspace

ROS 2 Humble workspace for the wheel-rail integrated greenhouse chassis.

This repository contains the navigation-side software only. The STM32 chassis
repository and all external reference projects are treated as read-only inputs.

## Target platform

- ROS 2 Humble on Ubuntu 22.04
- Gazebo Classic 11 for the first simulation baseline
- Four independently driven and independently steered wheel modules
- Steering range: approximately -165 to +165 degrees in the supplied model
- Four inner rail rollers driven coaxially with the ground tyres
- Livox MID-360 with built-in IMU
- Front and rear MaixCAM Pro cameras

## Package layout

| Package | Responsibility |
| --- | --- |
| `wr_interfaces` | Stable messages and services between replaceable modules |
| `wr_description` | Xacro robot model, meshes and sensor frames |
| `wr_swerve_controller` | Gazebo-side swerve kinematics, steering recentering and joint commands |
| `wr_rail_controller` | Rail clamp state machine, rail-mode drive and entry/exit transition policies |
| `wr_chassis_gateway` | Replaceable dry-run boundary for the future real chassis protocol |
| `wr_gazebo` | Gazebo worlds, controller configuration and simulation launch files |

Additional localization, rail-perception, mission and hardware-adapter packages
will be added behind the contracts in `wr_interfaces`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current status

The physical-model, flat-ground, on-rail, clamp, natural-entry and natural-exit
Gazebo baseline is implemented. The model uses the supplied 20 kg mass/COM/
inertia data and separate primitive collision bodies for tyres and inner rail
rollers. Real motor torque thresholds, sensor extrinsics and the physical
computer-to-STM32 protocol remain calibration/integration work; the real adapter
is intentionally dry-run until those values are supplied.
