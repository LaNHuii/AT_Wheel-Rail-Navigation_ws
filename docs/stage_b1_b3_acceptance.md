# Stage B1-B3 acceptance: rail support and low-speed travel

> Historical record, superseded on 2026-09-22 by
> `stage_b_physical_model_clamp_transition_acceptance.md`. In particular, the
> provisional 40 mm roller radius and 35 mm rail-centre height below must not be
> used as current model parameters.

Date: 2026-09-21

## Scope

This acceptance covers only a chassis already placed on two straight, parallel
rails. Natural entry/exit, reverse entry, sensing, localization, and autonomous
navigation are not part of this result.

## Geometry used

- rail radius: 25 mm
- rail center spacing: 600 mm
- rail center height above the concrete surface: 35 mm (provisional; measure the
  real installation and replace this value)
- simulated rail-roller radius: 40 mm
- simulated tire radius: 93.5 mm
- steering-module lateral offset for the rail configuration: 46 mm outward

The 46 mm value is derived from the four non-symmetric steering-module and
wheel transforms in `wr_chassis.urdf.xacro`. It places the four rail-roller
centers at approximately +299.9, -299.9, -300.6, and +299.9 mm instead of using
the tire-center track width as a proxy.

The 35 mm rail-center height gives about 6.5 mm nominal tire clearance. The
earlier 30 mm trial provided only 1.5 mm nominal clearance and allowed the tires
to regain floor contact during motion.

## Contact model

The tuned comparison world uses an ODE friction direction along the rail axis:

- longitudinal coefficient `mu = 0.60`
- lateral coefficient `mu2 = 20.0`
- rail-local `fdir1 = 0 0 1`, which maps to the world X rail direction

The high lateral coefficient is a simulation surrogate for the mechanical
lateral constraint and the planned steering-torque rail clamp. It must not be
treated as a measured steel-on-steel friction coefficient.

## Tests and results

### Static support

- base pose: z = 6.4 mm relative to the flat-ground spawn reference
- four tire-bottom clearances: 6.45-6.50 mm
- XY drift over 3 s: 0.82 mm
- vertical drift over 3 s: 0.01 mm
- yaw drift over 3 s: 0.0014 rad
- result: pass

### First low-speed segment

- command: `vx = 0.10 m/s` for 5 s
- measured travel: dx = 0.2501 m, dy = -0.0023 m
- yaw change: 0.0051 rad
- stop drift over 2 s: 7.80 mm
- final tire clearances: 3.98-6.19 mm
- result: pass

### Second consecutive segment without resetting Gazebo

- command: `vx = 0.10 m/s` for 5 s
- measured travel: dx = 0.2502 m, dy = 0.0001 m
- yaw change: 0.0115 rad
- stop drift over 2 s: 7.22 mm
- final tire clearances: 3.99-6.19 mm
- result: pass

## Failed trials retained as evidence

1. With a 38 mm module offset, the roller centers were about 8 mm inside each
   rail center. A 3.6 mm lateral shift caused the right tires to return to the
   floor.
2. With a 46 mm offset but a 30 mm rail-center height, lateral tracking was
   good, but all tires returned to the floor. Travel was 0.4886 m in 5 s,
   showing that propulsion had transferred back to the tires.

## Remaining qualifications

- The current swerve test controller converts body speed using the tire radius.
  Accurate rail-mode speed requires a separate rail controller or a mode-aware
  radius conversion using the 40 mm roller radius plus calibration.
- The 35 mm rail mounting height and lateral constraint must be replaced by
  measured real-chassis/rail values.
- The steering-torque clamp and its torque threshold are not implemented yet.
- Entry/exit transitions are Stage B4/B5 work.
