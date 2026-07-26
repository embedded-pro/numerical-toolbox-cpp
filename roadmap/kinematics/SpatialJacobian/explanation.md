# Spatial Jacobian — Overview

## What it is
The Jacobian is the matrix that links how fast the joints move to how fast the end-effector moves.
For an `N`-joint arm it is `6×N`: the top three rows give the tool's linear velocity, the bottom three
its angular velocity, as a linear function of the joint-rate vector `q̇`.

## Why it matters (embedded)
It is the workhorse of manipulator control. Velocity control, force control, singularity detection,
and every Jacobian-based inverse-kinematics solver need it. Its transpose maps end-effector
forces/torques back to joint torques — so the same matrix does velocity kinematics *and* statics,
exactly the reuse an embedded arm controller wants.

## How it works (intuition)
Each joint contributes one column. A **revolute** joint spins the tool about its own axis `zᵢ`, so it
adds angular velocity `zᵢ` and linear velocity `zᵢ × r`, where `r` is the lever arm from the joint to
the tool. A **prismatic** joint slides the tool along `zᵢ`, adding pure linear velocity `zᵢ` and no
rotation. Stack those columns and you have the map `twist = J·q̇`. When two columns line up the arm is
**singular** — it has locally lost a direction of motion, and `J` can no longer be inverted safely.

## Key parameters
- **joint axes `zᵢ`** and **origins `oᵢ`** — read off the forward-kinematics frame chain.
- **joint types** — revolute vs prismatic pick the column formula.
- **twist ordering** — `(v; ω)` (linear first) here; must be consistent everywhere downstream.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 5 (velocity kinematics and the Jacobian).

## See also
`DenavitHartenberg` (M7) / `ProductOfExponentials` (M15) supply the frames; `ManipulabilityIndex`
(M11) scores it; `PoseInverseKinematics` (M13) and `RedundancyResolution` (M14) invert it.
