# Fixed-front staged rail-entry acceptance

Date: 2026-09-24

## Implemented scope

- MID-360S frame pitched 45 degrees forward/downward.
- Forward-only entry policy plus a reusable 180-degree turnaround policy.
- Generic `RailClampCommand.module_mask`: front pair (`0x03`) or all modules (`0x0f`).
- Front-pair contact proof, selected-module release, rear-axle advance and final
  four-module proof.
- `ON_RAIL` is enabled only by final four-module proof.
- Gazebo entry controller executes `vx/vy/wz` using the same swerve math as the
  existing free-surface controller; the real chassis boundary remains `Twist`.

## Verification

- Pure controller/policy tests: 10 passed.
- Relevant ROS package build: 5 packages passed.
- Relevant package tests: 15 passed, 0 failures.
- Gazebo result: `STAGED_FIXED_FRONT_RAIL_ENTRY_VERIFICATION_PASSED`.
- Observed phases: `align`, `approach`, `front_probe`, `front_release`,
  `rear_advance`, `final_probe`, `confirmed`.

Gazebo uses virtual steering-contact events because its position controller does
not provide calibrated steering stall torque. Real-vehicle acceptance still
requires STM32 torque/current feedback, hard over-current protection and measured
contact thresholds. Entrance pose in this test comes from Gazebo ground truth; it
will later be replaced behind the same policy by MID-360S/camera perception.
