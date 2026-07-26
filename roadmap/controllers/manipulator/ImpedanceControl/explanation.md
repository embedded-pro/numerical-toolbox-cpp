# Impedance Control — Overview

## What it is
Instead of commanding a position, impedance control makes the end-effector *behave* like a chosen
mass–spring–damper. You program the stiffness, damping, and (optionally) inertia the robot presents
to the world — a tunable "softness" rather than a rigid trajectory.

## Why it matters (embedded)
Rigid position control shatters on contact: the tiniest position error against a hard surface
generates enormous force. Impedance control lets a robot push, insert, wipe, and physically interact
with people *safely*, with predictable and adjustable compliance. It is the foundation of
collaborative robots, assembly, and teleoperation.

## How it works (intuition)
Measure the Cartesian error between where the tip is and where it should be. Convert that error into
the force a virtual spring–damper would exert, add any commanded inertia and external-force term,
then use the Jacobian *transpose* to turn that tip force into joint torques. Compensating the arm's
own gravity and Coriolis terms ensures the felt impedance is the one you programmed — not the robot's
native dynamics. Because the mapping uses `Jᵀ` (never an inverse), it stays safe near singularities.

## Key parameters
- **Md (rendered inertia), Dd (damping), Kstiff (stiffness)** — the target impedance per Cartesian axis.
- **model, jacobian** — injected dynamics (`g`, `Cq̇`) and 6×N Jacobian.
- **fExternal** — optional measured contact wrench from a wrist force/torque sensor.

## Reference
N. Hogan, "Impedance Control: An Approach to Manipulation, Parts I–III,"
*ASME J. Dynamic Systems, Measurement, and Control*, 1985.

## See also
`OperationalSpaceControl` (needed to truly reshape inertia via `Λ`); `HybridPositionForceControl`
(partition force/motion axes); `ComputedTorqueControl` (rigid tracking counterpart).
