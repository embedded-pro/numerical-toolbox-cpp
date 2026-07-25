# Operational-Space Control — Overview

## What it is
A control framework that acts directly in Cartesian (task) space while correctly accounting for how
the arm's joint-space inertia *appears* at the end-effector — the task-space inertia `Λ`. Any leftover
joint freedom in a redundant arm is used by a secondary objective in the dynamically-consistent null
space.

## Why it matters (embedded)
It is the natural formulation for interaction and force control and for redundant (7-DOF) arms. By
reflecting the inertia through the Jacobian, it **dynamically decouples** task directions — a push in
`x` no longer disturbs `y` — and it lets a redundant arm avoid joint limits or obstacles while
rigidly holding a Cartesian pose.

## How it works (intuition)
Reflect the joint-space mass matrix through the Jacobian to obtain the effective end-effector inertia
`Λ = (J M⁻¹ Jᵀ)⁻¹`. Command a Cartesian acceleration with a task-space PD law, convert it to a wrench
by multiplying with `Λ`, and map that wrench to joint torque with `Jᵀ`. Finally, inject any secondary
joint torque through a null-space projector chosen so it is *invisible* to the task. The one delicate
step is the `M⁻¹Jᵀ` solve, handled by Gaussian elimination rather than an explicit inverse.

## Key parameters
- **Kp, Kd** — task-space position and damping gains.
- **model, jacobian** — injected dynamics and 6×N Jacobian.
- **tauSecondary** — secondary-objective joint torque (joint-limit / obstacle avoidance).
- **damping factor** — regularises `Λ` near singularities.

## Reference
O. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational
Space Formulation," *IEEE J. Robotics and Automation*, 3(1), 1987.

## See also
`ImpedanceControl` (`Λ` enables true inertia shaping); `HybridPositionForceControl` (adds force axes);
`RedundancyResolution` (kinematic null-space); `solvers/GaussianElimination` (the `M⁻¹Jᵀ` solve).
