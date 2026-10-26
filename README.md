# AT Wheel-Rail Navigation Workspace

ROS 2 Humble workspace for a person-following wheel-rail greenhouse carrier.

This repository contains the navigation-side software only. The STM32 chassis
repository and all external reference projects are treated as read-only inputs.

## Target platform

- ROS 2 Humble on Ubuntu 22.04
- Gazebo Classic 11 for the first simulation baseline
- Four independently driven and independently steered wheel modules
- Steering range: approximately -165 to +165 degrees in the supplied model
- Four inner rail rollers driven coaxially with the ground tyres
- Livox MID-360S point cloud, fixed 20 degrees forward/downward; its built-in
  IMU is not used as the primary localization IMU
- Chassis-mounted primary IMU sampled by STM32 and sent to the IPC over CAN-FD
- One front RGB-D camera (exact model pending) and one forward/downward MID-360S;
  only this sensor-front enters rails first

## Package layout

| Package | Responsibility |
| --- | --- |
| `wr_interfaces` | Stable messages and services between replaceable modules |
| `wr_description` | Xacro robot model, meshes and sensor frames |
| `wr_swerve_controller` | Gazebo-side swerve kinematics, steering recentering and joint commands |
| `wr_rail_controller` | Rail clamp state machine, rail-mode drive and entry/exit transition policies |
| `wr_chassis_gateway` | Dry-run/SocketCAN backends, CAN-FD codec and STM32 SDK contract |
| `wr_person_following` | Behavior-tree action endpoint and free/rail local follow state machine |
| `wr_lidar_safety` | MID-360S freshness, stop corridors and bounded free-surface bypass evidence |
| `wr_rail_perception` | Point-cloud rail pair detection, entrance pose and traversability estimate |
| `wr_localization` | Wheel odometry and chassis-IMU local EKF for control and trajectory audit |
| `wr_task_manager` | BehaviorTree.CPP task sequencing and explicit motion ownership transfer |
| `wr_gazebo` | Gazebo worlds, controller configuration and end-to-end workflow verification |

The RGB-D target producer remains owned by the vision group. Global mapping,
global route planning and automatic crop-row coverage are intentionally outside
the first person-following release.

## Build

```bash
tools/stage_build.sh
source /tmp/at_wheel_rail_navigation/install/setup.bash
tools/stage_test.sh
```

The stage scripts keep generated ROSIDL artifacts in an ASCII-only path and
select the Ubuntu system Python, avoiding the known non-ASCII-path and active
Conda conflicts on this development machine.

## Current status

The earlier physical-model, clamp and fixed-front staged-entry Gazebo baseline is
preserved. The current real-vehicle branch now has stable vision/safety/follow
interfaces, a bounded person-follow local state machine, conservative point-cloud
safety zones, a single-owner velocity arbiter, an IPC-to-STM32 CAN-FD adapter and
primary chassis-IMU transport. It also includes bounded free-surface bypass,
rail-entrance point-cloud estimation, wheel/IMU local odometry, an executable
top-level behavior tree and a Gazebo workflow verifier covering follow, staged
entry, rail motion, exit and remote takeover/rearm. The RGB-D target producer,
real torque thresholds, measured sensor extrinsics, CAN hardware bring-up and
fault-code calibration remain vehicle-integration work.
