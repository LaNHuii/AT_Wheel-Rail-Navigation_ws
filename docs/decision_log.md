# Decision log

## D001: ROS and simulator baseline

- ROS 2 Humble
- Gazebo Classic 11
- `gazebo_ros2_control` for joint-level physics

Reason: this is the installed and supported environment on the development PC.
The model and interfaces remain isolated so that migration to modern Gazebo is
possible later.

## D002: Mixed implementation language

- C++ for kinematics, controllers, point-cloud processing and hardware adapters.
- Python for launch, mission orchestration and experiment tooling.

## D003: Chassis boundary

- Standard free-surface input: `geometry_msgs/msg/Twist`.
- Standard feedback: `sensor_msgs/msg/JointState`, `nav_msgs/msg/Odometry` and
  `wr_interfaces/msg/SwerveState`.
- Rail engagement feedback: `wr_interfaces/msg/RailEngagementState`.
- Transport-specific CAN/serial details belong to a future hardware adapter.

## D004: Localization rollout

1. Validate model and controllers using simulation ground truth.
2. Validate wheel odometry as monitoring data, not the primary pose source.
3. Introduce simulated MID-360 and IMU.
4. Run FAST-LIO2 and compare against Gazebo truth.
5. Add global relocalization only after the local pose chain is measurable.

## D005: Provisional geometry

The supplied URDF contains visual meshes and joint locations but no inertial,
transmission, sensor or Gazebo elements. The navigation model may add primitive
collision and provisional inertial values while preserving the supplied visual
geometry. Every provisional value must remain configurable and documented.

