# SE(3) Transform, Twists and Wrenches — Overview

## What it is
The algebra of rigid-body motion: a transform is a rotation plus a translation; a *twist* is a rigid
body's instantaneous velocity (linear and angular); a *wrench* is a force together with a moment. The
adjoint map moves twists and wrenches between frames, and the exponential/logarithm maps convert
between a constant twist held for some time and the finite motion it produces.

## Why it matters (embedded)
Every pose-level algorithm — manipulator kinematics and Jacobians, pose inverse kinematics, Cartesian
interpolation, task-space control, drone attitude/position control, visual odometry — needs the same
small set of operations. Defining them once, with one ordering convention, prevents the silent sign
and ordering mismatches that otherwise appear when independently written modules exchange 6-vectors.

## How it works (intuition)
A transform stores a 3×3 rotation and a 3-vector translation; composing two transforms rotates and
shifts the second by the first, and the inverse has the closed form `(Rᵀ, −Rᵀp)`. A twist written in one
frame is re-expressed in another with the adjoint: rotate both parts, then add the "lever-arm" effect
of the frame offset on the linear part. Wrenches transform with the dual rule so that power (force
times velocity) is the same in every frame. The exponential turns "rotate about this line while
sliding along it" into a finite motion — Rodrigues' formula for the rotation plus a matching integral
for the translation — and the logarithm recovers that motion, which is exactly what pose-error
feedback needs.

## Key parameters
- **Ordering convention** — linear part first for twists `(v; ω)` and wrenches `(f; n)`.
- **Singular regions** — small angles and angles near 180° in the logarithm.
- **Orthonormality of `R`** — maintained on SO(3); re-orthonormalize against drift.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 3; R. M. Murray, Z. Li, S. S. Sastry,
*A Mathematical Introduction to Robotic Manipulation* (1994), Ch. 2; J. Solà, J. Deray, D. Atchuthan,
"A micro Lie theory for state estimation in robotics," arXiv:1812.01537 (2018). (The first two use
`(ω; v)` ordering; this library uses `(v; ω)`, which only permutes the blocks.)

## See also
`Quaternion` (the SO(3) rotation part), `Geometry3D` (`RotationAboutAxis`, `SkewSymmetric`),
`MatrixExponential` (the general matrix analogue of `Exp`).
