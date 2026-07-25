# Pose Inverse Kinematics — Overview

## What it is
Given a desired **pose** — position *and* orientation — of the tool, find joint angles that reach it.
This extends position-only IK to all six task dimensions by driving a 6-vector error (three for
translation, three for rotation) to zero with a damped-least-squares Newton step.

## Why it matters (embedded)
Real tasks are pose tasks: point a welding torch, align a gripper, hold a camera level. A numerically
robust solver that survives singularities without commanding infinite joint speeds is what makes an
arm safe to run in a tight real-time loop, warm-started from the previous cycle for a handful of
iterations.

## How it works (intuition)
Each step linearizes the arm about the current joints with the Jacobian, then asks "what joint change
best cancels the current pose error?" The plain least-squares answer, `J⁺e`, explodes near
singularities; adding a damping term `λ²I` inside the inverse tames it — the arm gives up a little
tracking accuracy in exchange for never blowing up. The subtle part is the **orientation** error:
subtracting Euler angles wraps and stalls, so the error is computed as the *rotation vector* (the log
of the relative rotation, or twice the vector part of the error quaternion), which always points the
short way around.

## Key parameters
- **damping λ** — larger = more stable near singularities, less accurate away from them.
- **tolerance** — the 6-vector error norm at which the solve stops.
- **max iterations** and **seed `q0`** — warm-starting from the last cycle keeps counts low.

## Reference
S. R. Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped
Least Squares methods," 2004; Y. Nakamura, H. Hanafusa, "Inverse Kinematic Solutions with Singularity
Robustness," *ASME J. Dyn. Sys. Meas. Control*, 1986.

## See also
`SpatialJacobian` (M8, the linearization), `Quaternion` (item 18, the orientation error),
`GaussianElimination` (the 6×6 solve), `AnalyticalIkPieper` (M21, the closed-form alternative).
