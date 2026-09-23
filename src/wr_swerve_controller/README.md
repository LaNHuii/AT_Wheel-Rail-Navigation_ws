# wr_swerve_controller

This package is a Gazebo and algorithm-test backend. It converts chassis-level
`vx`, `vy`, and `wz` commands into four steering-joint and four wheel-joint
commands because Gazebo has no STM32 lower controller.

It must not be launched in the real-vehicle configuration. The reference STM32
firmware already owns inverse swerve kinematics, equivalent-angle selection,
motor control, ramp limiting, and actuator-level fail-safe behavior. A real
transport adapter therefore forwards chassis-level commands to the MCU instead
of sending eight actuator commands from the industrial PC.
