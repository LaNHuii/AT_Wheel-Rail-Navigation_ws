# Architecture

## Design rules

1. Navigation packages never depend on the STM32 implementation.
2. `geometry_msgs/Twist` is the stable free-surface velocity command.
3. Simulation and real chassis communication are replaceable adapters.
4. Free-surface navigation and rail-constrained motion use separate controllers.
5. Rail engagement is an explicit state with observable evidence, not an
   inference from the planned path.
6. Unknown calibration values live in YAML/Xacro parameters.
7. Entry and exit use backend-independent transition policies; simulation
   ground truth and future perception are interchangeable observation sources.
8. Rail entry is fixed-front only: the 45-degree forward MID-360S and front RGB-D
   camera observe the entrance; the carrier turns around first when necessary.
9. The behavior tree owns mission sequencing; bounded local state machines own
   rail entry, exit and clamp execution. Only the chassis gateway accesses CAN.
10. The chassis-mounted IMU delivered by STM32 over CAN-FD is the primary IMU.
    The MID-360S IMU is not fused as a second primary source.

## Runtime boundaries

```text
RGB-D target ──> FollowPerson action / local follow FSM ──> /cmd_vel_follow
MID-360S ─────> local safety + obstacle map ──────────────> Nav2 bypass (later)
rail detector ─> ExecuteRailTransition / local FSM ───────> /cmd_vel_transition
                                      │
                     behavior tree selects one motion source
                                      ▼
                              cmd_vel_arbiter
                                      ▼
                          limit + watchdog + CAN adapter
                                      ▼
                                     STM32
```

The task layer follows the currently bound worker. It does not autonomously
select the next crop row. Track records are an audit aid and never imply that a
row was harvested merely because the chassis traversed it.

Perception and localization publish stable outputs independently of their
backends. This permits replacing FAST-LIO2, the rail detector, camera assistance,
or the final chassis protocol without changing mission logic.

## Coordinate convention

- `base_link`: x forward, y left, z up.
- Steering angle zero: wheel rolling direction parallel to +x.
- Positive steering angle: counter-clockwise about +z.
- Wheel order: front-left, front-right, rear-right, rear-left.
- Rail entry is allowed only along `base_link` +x (the fixed sensor-front).
- Normal completion exits forward. Automatic blocked-row retreat is disabled
  until rearward perception is added and separately accepted.

## Parameters awaiting measurement

- Effective ground tyre rolling radius.
- Effective inner rail-roller rolling radius.
- Measured MID-360S, chassis IMU and front RGB-D camera extrinsics (MID-360S pitch is
  fixed at 45 degrees).
- Motor-side gear ratios and sign conventions.
- Steering torque bias, contact thresholds and temperature dependence.
- Maximum admissible entry lateral and heading errors.

The supplied `robot(1).zip` is now authoritative for per-link mass, centre of
mass and inertia; the checked model totals 20 kg. Steering travel is modelled as
approximately +/-165 degrees. Tyre and inner-roller primitive collision sizes
are derived from the supplied STL bounds and remain subject to real measurement.
