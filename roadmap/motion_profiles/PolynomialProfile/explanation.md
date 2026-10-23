# Polynomial Point-to-Point Profile — Overview

## What it is
A closed-form time law that drives an axis from a start state to a goal state over a fixed
duration. A **cubic** matches endpoint position and velocity; a **quintic** additionally matches
endpoint acceleration, giving a smoother (bounded-jerk) motion.

## Why it matters (embedded)
Point-to-point moves are the bread and butter of servo motion — robot joints, CNC and pick-and-place
axes, motor position loops. The coefficients are solved **once** from a handful of algebraic
expressions, after which each servo tick is just a Horner polynomial evaluation — no iteration, no
lookup table, fully deterministic cycle count.

## How it works (intuition)
A degree-`n` polynomial has `n+1` free coefficients. A cubic (4 coefficients) exactly satisfies
four boundary conditions — start/end position and start/end velocity. A quintic (6 coefficients)
adds start/end acceleration. Because the boundary equations are linear in the coefficients and the
time basis is fixed, the solution is a plug-in formula rather than a matrix solve.

## Key parameters
- **Boundary conditions** — start/end position, velocity, and (quintic) acceleration.
- **Duration `tf`** — the move time; sets the peak velocity/acceleration for a given distance.
- **Degree** — cubic (velocity-matched) or quintic (acceleration-matched, jerk-limited endpoints).

## Reference
M. W. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Ch. 5 (polynomial
trajectories).

## See also
`TrapezoidalProfile` (limit-respecting, not fixed-time), `SCurveProfile` (jerk-bounded),
`CubicSpline` (#61, many via points, C² — reduces to this cubic for two knots); robotics-toolbox-cpp's
`CartesianSlerpInterpolation` (task-space paths; a 0 → 1 polynomial can drive its time law).
