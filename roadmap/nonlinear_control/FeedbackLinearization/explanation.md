# Feedback Linearization — Overview

## What it is
A control technique that **cancels** a plant's known nonlinear dynamics with an inner control law,
leaving an equivalent linear system that a simple outer loop (PD, LQR) can drive. For mechanical
systems this is the *computed-torque* method: `τ = M(q)·v + C(q,q̇)q̇ + g(q)` turns the robot into
a set of decoupled double integrators `q̈ = v`.

## Why it matters (embedded)
Robot arms, quadrotors, and other structurally-known machines are strongly nonlinear — a fixed PID
tuned at one operating point misbehaves at another. Feedback linearization uses the *model you
already have* (`M`, `C`, `g` from `dynamics/`) to erase that nonlinearity, so one linear gain set
works across the whole workspace. No gain scheduling, no lookup tables.

## How it works (intuition)
Split the controller in two. The **inner** law evaluates the model at the current state and injects
exactly the torque needed to cancel gravity, Coriolis, and inertia coupling. What remains behaves
like unit masses. The **outer** law then commands a virtual acceleration `v = q̈_d + Kd·ė + Kp·e`
as if controlling those trivial masses. Because the cancellation multiplies by `M(q)` (never
inverts it), the hot path stays well-conditioned.

## Key parameters
- **nonlinear model** — injected `EulerLagrangeDynamics` supplying `M(q)`, `C(q,q̇)q̇`, `g(q)`.
- **Kp, Kd** — outer-loop gains for the linearized double integrator; pick `Kd = 2√Kp` for
  critical damping.
- **relative degree** (general form) — how many times to differentiate the output before the input
  appears; sets the structure of the cancellation.

## Reference
A. Isidori, *Nonlinear Control Systems*, 3rd ed. (1995); J.-J. Slotine, W. Li,
*Applied Nonlinear Control* (1991), computed-torque chapter.

## See also
`BacksteppingControl` (recursive alternative that tolerates non-cancellable terms);
`ModelReferenceAdaptiveControl` (adapts the model online when parameters are unknown);
`dynamics/EulerLagrangeDynamics` (the injected model); `Lqr` (a natural outer loop).
