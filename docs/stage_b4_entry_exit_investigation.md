# Stage B4 investigation: natural rail entry and exit

> Historical investigation, superseded on 2026-09-22 by
> `stage_b_physical_model_clamp_transition_acceptance.md`. It is retained as
> failed-trial evidence; its provisional roller/rail geometry is obsolete.

Date: 2026-09-21

Status: **not accepted / parameters not frozen**

## What was built

- `rail_entry.world`: concrete approach, two provisional 7.1% circular-rail
  lead-in ramps, and two 600 mm-center-spacing main rails.
- `rail_entry.launch.py`: starts the rail-entry scenario with the same guarded
  `/cmd_vel` interface used by the flat-ground and on-rail tests.
- Open-loop, simultaneous-feedback, stop-and-align, phased-entry, and reverse
  exit verification scripts.
- `rail_simulation.py`: shared launch construction for later rail scenarios.

Gazebo ground truth is used only as a temporary substitute for the future
camera/point-cloud entrance-pose estimate. None of these verification scripts
is the final perception implementation.

## Algorithm comparisons performed

### Open-loop `vx` only

Result: rejected. The free-ground chassis accumulated 21 mm lateral error and
one roller reached the entrance before the other. The vehicle stopped at the
entrance instead of transferring to four-roller support.

### Simultaneous `vx + vy + wz` proportional correction

Result: rejected. Once one side contacted the rail, continuing lateral/yaw
correction amplified the contact asymmetry. Peak lateral error reached 154 mm.

### Stop-and-align state machine

Result: provisionally successful once, but not repeatable. One run achieved:

- entry travel: 1.5115 m
- peak lateral error: 27.5 mm
- peak yaw error: 0.0697 rad
- final four tire clearances: 6.48-6.50 mm
- stop drift: 2.35 mm

Later clean-world repetitions stopped with only the front rollers supported.
Therefore the single successful run is evidence that the geometry can work,
not an acceptance result.

### Phased `align -> approach -> vx-only transition`

This is the preferred control architecture because it enforces `vy = 0` and
`wz = 0` after rail contact. In the current provisional mechanical scene it
kept pre-contact error small (about 20 mm lateral and 0.035 rad yaw) but still
showed non-repeatable single-side contact at the lead-in.

### Reverse exit

The successful-entry state was reversed through the same lead-in:

- reverse travel: 1.3025 m
- all four tires returned to the concrete
- stop drift: 0.53 mm
- peak lateral error: 54.9 mm

The physical support transition succeeded, but the 54.9 mm lateral excursion
exceeded the provisional 50 mm criterion. It was therefore not marked pass.

## Why Stage B4 is not accepted

The horizontal on-rail geometry is repeatable, but entry behavior is dominated
by values that are not yet measurements of the real machine:

- the rail center height above the concrete is provisionally 35 mm;
- the simulated roller radius is provisionally 40 mm;
- the lead-in ramp length/profile is invented for the simulation;
- the steering-torque clamp is not represented;
- the actual rail end cap, bevel, or guide structure is unknown.

Changing controller thresholds cannot establish physical validity while these
mechanical boundary conditions are unknown.

## Data needed before freezing B4

1. Rail center height (or rail top height) relative to the adjacent concrete.
2. Close side/top photographs or measurements of the rail entrance end:
   vertical cut, rounded end, bevel, or separate guide ramp.
3. Actual inner roller diameter and width.
4. Approximate steering clamp command after contact: target angle, motor
   current/torque, or at least the intended direction for each module.

With these values, replace the provisional lead-in and implement the planned
torque-clamp mode before repeating entry/exit acceptance.
