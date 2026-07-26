# Generic Joint Link — Overview

## What it is
A single struct that describes **either** a revolute (rotating) **or** a prismatic (sliding) joint,
by carrying a joint *type* alongside a unit *axis* and the usual inertial parameters. It generalizes
the revolute-only link so one chain can mix hinges and slides.

## Why it matters (embedded)
Real machines are rarely all-revolute: SCARA arms, gantry/Cartesian robots, hydraulic rams, and
3-D printers all contain sliding axes. Encoding the joint type once — as a byte-sized enum plus a
shared code path — lets forward kinematics, the Jacobian, and inverse dynamics handle mixed chains
without duplicating an entire link/algorithm family per joint type.

## How it works (intuition)
Every 1-DOF joint moves along a **screw axis**. A revolute joint spends its motion in the *angular*
part of that screw (it rotates by `q` about the axis); a prismatic joint spends it in the *linear*
part (it slides by `q` along the axis). The link therefore exposes two things: the relative
transform produced by `q`, and which channel — angular or linear — the axis occupies. Downstream
algorithms read those and stay joint-type-agnostic.

## Key parameters
- **type** — `Revolute` or `Prismatic`.
- **axis** — unit vector; rotation axis (revolute) or slide direction (prismatic).
- **parentToJoint / jointToCoM** — link geometry, unchanged from the revolute link.
- **mass / inertia** — rigid-body inertial parameters.

## Reference
J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3
(link description and joint transforms).

## See also
`RevoluteJointLink` (the specialization it generalizes), `RecursiveNewtonEuler` (consumes the
motion subspace), Denavit-Hartenberg parameters (M7), spatial Jacobian (M8).
