# PD + Gravity Compensation — Overview

## What it is
The simplest globally-stable manipulator set-point regulator: a PD loop on the joint error plus an
exact gravity-cancelling feedforward term — `τ = Kp·e − Kd·q̇ + g(q)`.

## Why it matters (embedded)
It is the cheapest *model-based* joint controller in robotics. It needs only the gravity vector
`g(q)` — not the full mass matrix or Coriolis terms — so it runs in a handful of flops on a modest
MCU, yet it drives the arm to any pose with **zero steady-state droop** and a proof of global
stability. Ideal for hold/positioning tasks and as a safe fallback controller.

## How it works (intuition)
The PD term behaves like a virtual spring-damper pulling each joint toward its target and bleeding
off velocity. Left alone, that spring would sag under gravity and settle short of the goal. The
`g(q)` feedforward supplies exactly the torque needed to counter gravity at every configuration, so
the spring no longer fights it and the joint lands precisely on the set-point. A Lyapunov
energy argument (kinetic + spring potential) guarantees the arm always converges.

## Key parameters
- **model** — injected dynamics object supplying the gravity term `g(q)`.
- **Kp (stiffness)** — how hard the controller pulls toward the target.
- **Kd (damping)** — how strongly it resists velocity; pick `Kd ≈ 2√(Kp·inertia)` for critical damping.

## Reference
M. Takegaki, S. Arimoto, "A New Feedback Method for Dynamic Control of Manipulators,"
*ASME J. Dynamic Systems, Measurement, and Control*, 1981.

## See also
`ComputedTorqueControl` (full inverse-dynamics tracking); `ImpedanceControl` (compliant contact);
`FrictionCompensation` (adds a friction feedforward); `dynamics/EulerLagrangeDynamics` (the injected model).
