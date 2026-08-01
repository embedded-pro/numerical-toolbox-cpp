# Feedback Linearization — Overview

## What it is
A control technique that **cancels** a control-affine plant's known nonlinear dynamics with an inner
control law, leaving an equivalent linear system that a simple outer loop (PD, LQR) can drive. For a
system whose input enters through a state-dependent decoupling matrix, the cancelling law is
`u = B(x)·v + a(x)`, which turns the plant into decoupled integrator chains `ÿ = v`. The mechanical
*computed-torque* method `τ = M(q)·v + C(q,q̇)q̇ + g(q)` is the canonical instance (`B = M`,
`a = Cq̇ + g`).

## Why it matters (embedded)
Robot arms, quadrotors, and other structurally-known machines are strongly nonlinear — a fixed PID
tuned at one operating point misbehaves at another. Feedback linearization uses the *model you
already have* to erase that nonlinearity, so one linear gain set works across the whole operating
envelope. No gain scheduling, no lookup tables.

## How it works (intuition)
Split the controller in two. The **inner** law evaluates the injected model at the current state and
injects exactly the input needed to cancel the drift `a(x)`. What remains behaves like plain
integrator chains. The **outer** law then commands a virtual input `v = y_d^{(r)} + Kd·ė + Kp·e`
as if controlling those trivial integrators. In the mechanical (input-state) case the cancellation
multiplies by `B(x) = M(q)` — never inverts it — so the hot path stays well-conditioned.

## Key parameters
- **nonlinear model** — an injected control-affine model supplying the decoupling matrix `B(x)` and
  the drift term `a(x)` to be cancelled. (The manipulator instance — `M(q)`, `C(q,q̇)q̇`, `g(q)` —
  lives in robotics-toolbox-cpp.)
- **Kp, Kd** — outer-loop gains for the linearized integrator chain; pick `Kd = 2√Kp` for
  critical damping.
- **relative degree** (general form) — how many times to differentiate the output before the input
  appears; sets the structure of the cancellation.

## Reference
A. Isidori, *Nonlinear Control Systems*, 3rd ed. (1995); J.-J. Slotine, W. Li,
*Applied Nonlinear Control* (1991), computed-torque chapter.

## See also
`BacksteppingControl` (recursive alternative that tolerates non-cancellable terms);
`ModelReferenceAdaptiveControl` (adapts the model online when parameters are unknown);
`Lqr` (a natural outer loop). The manipulator computed-torque instantiation is
`controllers/manipulator/ComputedTorqueControl` in robotics-toolbox-cpp.
