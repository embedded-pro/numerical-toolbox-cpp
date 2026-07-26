# S-Curve (Jerk-Limited) Profile — Overview

## What it is
A **seven-segment, jerk-bounded** motion profile (a "double-S" velocity curve). It extends the
trapezoid by ramping acceleration in and out with a bounded rate of change (jerk), so acceleration
is continuous rather than stepping instantaneously.

## Why it matters (embedded)
Sudden acceleration jumps excite structural resonances, wear gearing, and spill liquids or wobble
payloads. Limiting **jerk** smooths those transitions, cutting vibration and mechanical stress at
the cost of a slightly longer move. It is the profile of choice for high-precision or
vibration-sensitive machines (pick-and-place, CNC, semiconductor handling).

## How it works (intuition)
The velocity curve is built from seven phases: jerk up, hold acceleration, jerk down (to reach
cruise velocity), cruise, then the mirror-image deceleration triple. Depending on the distance and
limits, some phases shrink to zero — a short move may never reach `aMax` or `vMax`. All phase
durations follow from closed-form algebra on the velocity/acceleration/jerk ceilings.

## Key parameters
- **`vMax`** — cruise-velocity ceiling.
- **`aMax`** — acceleration ceiling (flat top of the acceleration curve).
- **`jMax`** — jerk ceiling; the slope of the acceleration ramps and the vibration knob.

## Reference
L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008),
Ch. 3 (double-S / jerk-limited profiles).

## See also
`TrapezoidalProfile` (jerk-unbounded predecessor), `PolynomialTrajectory` (fixed-time quintic),
`TimeOptimalPathParameterization` (limits along a geometric path).
