# Analytical IK (Pieper) — Overview

## What it is
A **closed-form** inverse-kinematics solver for the classic six-revolute arm whose last three axes
meet at a point (a spherical wrist). Instead of iterating, it computes every joint angle directly with
trigonometry and returns *all* the arm postures — up to eight — that reach a given pose.

## Why it matters (embedded)
Closed-form IK is exact, has no convergence loop, and runs in constant time — ideal for a hard
real-time controller that cannot afford an iterative solver's worst-case iteration count. Getting *all*
solutions also lets a planner choose the posture that best avoids joint limits or obstacles, something
a single-answer numerical solver cannot offer.

## How it works (intuition)
Pieper's insight is that a spherical wrist **decouples** the problem. The wrist centre — the point
where the last three axes cross — depends only on the first three joints, so you first back it out of
the target pose and solve a 3-DOF *position* problem for joints 1-3 using plain planar geometry (an
`atan2` for the base and the law of cosines for the elbow, giving shoulder and elbow-up/down branches).
With the arm placed, the leftover rotation from frame 3 to the tool is handled by wrist joints 4-6,
extracted as a set of Euler angles (two branches for the middle wrist joint). Multiplying the branch
counts gives up to eight complete solutions.

## Key parameters
- **DH table of the 6R arm** — with the spherical-wrist assumption baked in.
- **target pose** — the tool position and orientation to invert.
- **branch selection** — shoulder, elbow, and wrist flips the caller chooses among.

## Reference
D. L. Pieper, "The Kinematics of Manipulators Under Computer Control," PhD thesis, Stanford
University, 1968.

## See also
`DenavitHartenberg` (M7) / `SE3Transform` (M6) for the frames, `Cordic` (item 23) for the trig,
`PoseInverseKinematics` (M13, the iterative fallback for non-spherical wrists).
