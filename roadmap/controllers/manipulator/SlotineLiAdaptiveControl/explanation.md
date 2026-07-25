# Slotine-Li Adaptive Control — Overview

## What it is
A trajectory-tracking manipulator controller that **learns the robot's inertial parameters online**
while it moves. It exploits the fact that rigid-body dynamics are *linear in the inertial
parameters*: `M(q)q̈ + C(q,q̇)q̇ + g(q) = Y(q,q̇,q̈)·a`, so the unknown masses/inertias `a` can be
estimated by a simple adaptation law.

## Why it matters (embedded)
Real robots carry unknown or changing payloads. Rather than re-identifying the model offline, this
controller adapts its feedforward in real time, giving accurate tracking without precise a-priori
mass data — valuable when the same low-cost arm must handle varied loads.

## How it works (intuition)
It combines a *sliding variable* `s = ė + Λe` (a filtered tracking error) with a regressor `Y`
evaluated at a **reference** acceleration, so no noisy joint-acceleration measurement is needed. The
torque is a model feedforward `Y·â` plus damping `−Kd·s`. Whenever `s` is nonzero the parameter
estimate `â` slides *downhill* on a Lyapunov function (`â̇ = −Γ Yᵀ s`), simultaneously shrinking the
tracking error and improving the model. Passivity guarantees stability.

## Key parameters
- **Λ (surface slope)** — bandwidth of the `s = ė + Λe` error manifold.
- **Kd (sliding gain)** — damping on `s`; sets transient stiffness.
- **Γ (adaptation gain)** — how fast parameters are learned; too large ⇒ oscillation.
- **â₀ (initial estimate)** — parameter seed; a rough guess speeds convergence.

## Reference
J.-J. Slotine, W. Li, "On the Adaptive Control of Robot Manipulators," *Int. J. Robotics Research*,
6(3), 1987.

## See also
`ComputedTorqueControl` (#M12, exact-model sibling), `ModelReferenceAdaptiveControl` (#47),
`DynamicParameterIdentification` (#M22, offline regressor identification).
