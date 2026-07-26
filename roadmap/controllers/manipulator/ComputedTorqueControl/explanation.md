# Computed-Torque Control — Overview

## What it is
The workhorse model-based *tracking* law for manipulators. An inverse-dynamics feedforward plus a
PD correction linearises and decouples the arm into independent unit double integrators:
`τ = M(q)(q̈_d + Kd·ė + Kp·e) + C(q,q̇)q̇ + g(q)`.

## Why it matters (embedded)
A fixed PID tuned at one posture misbehaves at another because a robot's inertia and gravity change
with configuration. Computed-torque uses the `M`, `C`, `g` you already evaluate with RNEA to erase
that variation, so a *single* gain set tracks fast trajectories across the entire workspace — no
gain scheduling, no lookup tables — at a deterministic `O(n)` cost.

## How it works (intuition)
Work out the acceleration you actually want: the desired trajectory acceleration plus a PD term that
corrects position and velocity error. Then ask the dynamics model, "what joint torque produces
exactly that acceleration *right now*?" Because the answer multiplies by the mass matrix `M(q)`, the
arm's inertial coupling, Coriolis, and gravity are all cancelled, leaving clean, identical
second-order error dynamics on every joint.

## Key parameters
- **model** — injected dynamics supplying `M(q)`, `C(q,q̇)q̇`, `g(q)`.
- **Kp, Kd** — error gains for the linearised double integrator; pick `Kd = 2√Kp` for critical damping.

## Reference
M. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Ch. 8; Luh, Walker, Paul (1980).

## See also
`PdGravityCompensation` (set-point-only special case); `FeedbackLinearization` (same idea for general
plants); `SlotineLiAdaptiveControl` (adapts unknown parameters); `dynamics/RecursiveNewtonEuler` (term source).
