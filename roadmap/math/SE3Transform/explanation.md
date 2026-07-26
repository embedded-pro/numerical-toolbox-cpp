# SE(3) Transform — Overview

## What it is
`SE(3)` is the group of rigid-body motions in 3D: a rotation plus a translation, stored as a 4×4
homogeneous matrix `[[R, p],[0, 1]]`. Its "velocity" companions are **twists** (6-vectors pairing
angular and linear velocity) and **wrenches** (moment plus force); the **adjoint** map moves those
6-vectors between coordinate frames.

## Why it matters (embedded)
This is the algebra every modern manipulator, drone, and SLAM stack is built on. Forward kinematics
chains `SE(3)` products; Jacobians and dynamics are expressed with twists and the adjoint; trajectory
interpolation lives on this group. A compact, correct `SE(3)` type is the foundation the whole
robotics layer reuses.

## How it works (intuition)
Composing two motions multiplies their homogeneous matrices, which reduces to `R = R₁R₂`,
`p = R₁p₂ + p₁` — no full 4×4 multiply needed. The inverse has the closed form `(Rᵀ, −Rᵀp)`. The
**exponential map** turns a constant twist (a screw axis) applied for a "time" `θ` into the finite
motion it produces — the rigid-body generalization of Rodrigues' rotation formula — and its inverse
**log** recovers the screw from a transform. The **adjoint** is the 6×6 that re-expresses a twist or
wrench in another frame.

## Key parameters
- **Twist ordering** — `(ω; v)` (angular first) here; the adjoint layout follows this convention.
- **Screw parameter `θ`** — how far along a twist/screw axis the exponential integrates.
- **Orthonormality of `R`** — must be maintained on `SO(3)`; reorthonormalize against drift.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 3; R. M. Murray, Z. Li, S. S. Sastry,
*A Mathematical Introduction to Robotic Manipulation* (1994).

## See also
`Quaternion` (item 18, the `SO(3)` rotation part), `Geometry3D` (`RotationAboutAxis`,
`SkewSymmetric`), `MatrixExponential` (item 29, the `se(3)→SE(3)` analogue), `DenavitHartenberg` (M7).
