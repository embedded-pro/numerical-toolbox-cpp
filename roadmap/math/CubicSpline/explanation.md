# Cubic Spline Interpolation — Overview

## What it is
A smooth curve through a list of points: between consecutive knots each channel is a cubic
polynomial, and the pieces join so value, slope **and** curvature are continuous (C²). The ends are
either **clamped** (prescribed slopes) or **natural** (zero curvature). Several channels that share the
same knots are fitted together.

## Why it matters (embedded)
Waypoint lists (robot teach-in programs, CNC paths, planner output), calibration tables and resampled
signals all need a smooth interpolant that does not ring like a high-order polynomial. Fitting is one
linear-time pass on fixed storage; each evaluation is a binary search plus a cubic — cheap and
deterministic.

## How it works (intuition)
The unknowns are the curvatures at the knots. Once known, each segment is fixed: it must hit both knot
values and match both knot curvatures. Requiring neighbouring segments to have the same slope at every
interior knot gives one equation per knot that involves only the knot and its two neighbours — a
**tridiagonal** system. The end conditions add the first and last equations. The system is diagonally
dominant, so the Thomas algorithm solves it in linear time without pivoting, and its elimination
factors depend only on the knot spacing, so every channel reuses them.

## Key parameters
- **Knots `x_k`** — strictly increasing abscissae (times, arc length, sensor readings).
- **Values `y_k`** — one vector per knot.
- **End condition** — clamped (end slopes, default zero) or natural (zero end curvature).
- **`MaxKnots`** — compile-time capacity of the bounded storage.

## Reference
C. de Boor, *A Practical Guide to Splines*, rev. ed. (2001), Ch. IV; L. Biagiotti, C. Melchiorri,
*Trajectory Planning for Automatic Machines and Robots* (2008), Ch. 4.

## See also
`TridiagonalSolver` (#59, the solve), `PolynomialProfile` (#55, the two-knot special case),
`PolynomialFitting` (least-squares fit rather than interpolation); robotics-toolbox-cpp's joint-space
spline trajectory, a thin wrapper over this interpolant.
