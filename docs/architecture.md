# Architecture

## Design rules

1. Navigation packages never depend on the STM32 implementation.
2. `geometry_msgs/Twist` is the stable free-surface velocity command.
3. Simulation and real chassis communication are replaceable adapters.
4. Free-surface navigation and rail-constrained motion use separate controllers.
5. Rail engagement is an explicit state with observable evidence, not an
   inference from the planned path.
6. Unknown calibration values live in YAML/Xacro parameters.
7. Entry and exit share a backend-independent transition policy; simulation
   ground truth and future perception are only interchangeable observation
   sources.

## Runtime boundaries

```text
Mission / coverage planner
          |
          +---- free surface ----> Nav2 ----> /cmd_vel
          |
          +---- rail entry ------> rail entry controller
          |
          +---- on rail ---------> rail longitudinal controller
                                          |
                                chassis command adapter
                                  /                 \
                           Gazebo adapter       STM32 adapter
```

Perception and localization publish stable outputs independently of their
backends. This permits replacing FAST-LIO2, the rail detector, camera assistance,
or the final chassis protocol without changing mission logic.

## Coordinate convention

- `base_link`: x forward, y left, z up.
- Steering angle zero: wheel rolling direction parallel to +x.
- Positive steering angle: counter-clockwise about +z.
- Wheel order: front-left, front-right, rear-right, rear-left.
- Rail entry can be performed from the front or rear; observations are always
  transformed into `base_link` before control.

## Parameters awaiting measurement

- Effective ground tyre rolling radius.
- Effective inner rail-roller rolling radius.
- MID-360 and both camera extrinsics.
- Motor-side gear ratios and sign conventions.
- Steering torque bias, contact thresholds and temperature dependence.
- Maximum admissible entry lateral and heading errors.

The supplied `robot(1).zip` is now authoritative for per-link mass, centre of
mass and inertia; the checked model totals 20 kg. Steering travel is modelled as
approximately +/-165 degrees. Tyre and inner-roller primitive collision sizes
are derived from the supplied STL bounds and remain subject to real measurement.
