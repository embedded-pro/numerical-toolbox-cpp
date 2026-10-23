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
limits, some phases shrink to zero — a short move may never reach `aMax` or `vMax`. For rest-to-rest
moves the phase durations follow from a three-step closed form: assume `vMax` is reached; if the move is
too short, drop the cruise and assume only `aMax` is reached; if still too short, neither is reached and
the acceleration ramps are pure triangles. Several axes finish together by stretching the faster ones in
time by a common factor `λ` (velocity `/λ`, acceleration `/λ²`, jerk `/λ³`).

## Key parameters
- **`vMax`** — cruise-velocity ceiling.
- **`aMax`** — acceleration ceiling (flat top of the acceleration curve).
- **`jMax`** — jerk ceiling; the slope of the acceleration ramps and the vibration knob.

## Reference
L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008),
§3.4 (double-S / jerk-limited profiles; the non-zero boundary-velocity case there needs an iterative
`aMax` reduction and is future work).

## See also
`TrapezoidalProfile` (jerk-unbounded predecessor), `PolynomialProfile` (fixed-time quintic),
robotics-toolbox-cpp's `TimeOptimalPathParameterization` (limits along a geometric path).
