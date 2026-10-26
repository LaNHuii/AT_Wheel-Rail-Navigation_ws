# Decision log

## D001: ROS and simulator baseline

- ROS 2 Humble
- Gazebo Classic 11
- `gazebo_ros2_control` for joint-level physics

Reason: this is the installed and supported environment on the development PC.
The model and interfaces remain isolated so that migration to modern Gazebo is
possible later.

## D002: Mixed implementation language

- C++ for kinematics, hard real-time-adjacent guards and hardware adapters.
- Python/NumPy for the initial point-cloud safety baseline; profile it on the IPC
  before deciding whether the production implementation must move to C++.
- Python for launch, mission orchestration and experiment tooling.

## D003: Chassis boundary

- Standard free-surface input: `geometry_msgs/msg/Twist`.
- Standard feedback: `sensor_msgs/msg/JointState`, `nav_msgs/msg/Odometry` and
  `wr_interfaces/msg/SwerveState`.
- Rail engagement feedback: `wr_interfaces/msg/RailEngagementState`.
- Transport-specific CAN-FD details belong only to `wr_chassis_gateway`.

## D004: Localization rollout

1. Validate model and controllers using simulation ground truth.
2. Fuse wheel odometry and the chassis primary IMU for short-term local pose.
3. Record local pose for controller support and post-task trajectory audit.
4. Add FAST-LIO2/global relocalization only if later experiments show that
   person-led operation requires a globally consistent map.

## D005: Provisional geometry

The supplied URDF contains visual meshes and joint locations but no inertial,
transmission, sensor or Gazebo elements. The navigation model may add primitive
collision and provisional inertial values while preserving the supplied visual
geometry. Every provisional value must remain configurable and documented.


## D006: Fixed-front rail entry and staged proof

- MID-360S is nominally fixed at 20 degrees forward/downward.
- Only the mechanical sensor-front may enter a rail first; mission planning must
  insert a 180-degree turn when the next entrance faces the opposite direction.
- One generic clamp command is reused with a module bitmask instead of creating
  phase-specific hardware APIs.
- Entry proof order is front pair, release, rear advance, then all four modules.
- Front-pair success is intermediate evidence and never enables `ON_RAIL`.
- Normal exit is forward. Reverse exit remains in the compatibility interface,
  but automatic retreat is disabled while no rearward perception is installed.

## 2026-10-08：任务转为人物跟随采收运输

- 指定采摘者负责带路；系统不再根据覆盖清单自主选择下一条轨道。
- 视觉使用一个前置 RGB-D 相机和一台前向下倾 MID-360S；取消后置相机模型。
- 自由路面允许跟随、车头转向和有界绕障；轨道内只允许纵向跟随。
- 首版目标丢失只停车等待，不实现主动搜索和自动换人；这些能力留待视觉身份链稳定后增加。
- 行为树负责任务编排和运动权切换，人物跟随、点云安全及进退轨各自使用局部状态机。

Reason: one forward 3D lidar avoids a second lidar while keeping entrance
geometry observable. The current vehicle has one front RGB-D camera and no
rearward perception, so reverse autonomous motion is outside the accepted scope.

## 2026-10-15：MID-360S 安装倾角调整

- MID-360S 的标称安装姿态由向前下倾 45°调整为向前下倾 20°。
- 固定传感器前端进轨、单雷达和前向退出策略保持不变。
- 20°仅是当前机械安装标称值；实车外参标定结果作为点云转换的最终依据。

Reason: 20°安装姿态在保留近地轨道观测能力的同时扩大前向障碍物与人员周边环境的有效视野。
