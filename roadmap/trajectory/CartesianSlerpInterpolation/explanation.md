# Cartesian Path + Orientation (SLERP) Interpolation — Overview

## What it is
A task-space motion generator: it moves the end-effector along a **straight line** (or screw) in
Cartesian position while blending orientation with **SLERP** (spherical linear interpolation of
unit quaternions). A single scalar progress variable `s(t) ∈ [0, 1]` drives both channels so they
start and finish together.

## Why it matters (embedded)
Many manipulator tasks are defined in the workspace, not joint space — draw a straight bead, keep a
tool square to a surface, approach along an axis. Interpolating position and orientation directly in
Cartesian space produces predictable, collision-friendly paths that joint-space interpolation cannot
guarantee.

## How it works (intuition)
Position is a plain linear blend between the two endpoints. Orientation is trickier: naively
averaging quaternions leaves the sphere and changes speed. SLERP instead walks the **great-circle
arc** on the unit sphere at constant angular rate, giving the shortest, smoothest rotation. Feeding
`s(t)` from a trapezoidal or polynomial time law shapes how fast the pose advances along the path.

## Key parameters
- **Start / goal pose** — Cartesian position plus a unit quaternion orientation.
- **Duration `tf`** — total move time.
- **Time scaling `s(t)`** — linear, trapezoidal, or polynomial progress law.

## Reference
K. Lynch, F. Park, *Modern Robotics* (2017), Ch. 9 (trajectory generation);
K. Shoemake, "Animating rotation with quaternion curves," *SIGGRAPH* 1985 (SLERP).

## See also
`Quaternion` (item #18, provides `Slerp`), `SE3Transform` (item #M6),
`TrapezoidalProfile` / `PolynomialTrajectory` (the `s(t)` drivers).
