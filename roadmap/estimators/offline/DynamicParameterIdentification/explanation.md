# Dynamic Parameter Identification — Overview

## What it is
An offline (batch) estimator that recovers a manipulator's **link inertial parameters** — masses,
centers of mass, and inertia tensors (in their identifiable *base-parameter* combinations) — from
recorded motion. It exploits the fact that the inverse-dynamics torque is **linear in these
parameters**: `Y(q, q̇, q̈)·a = τ`.

## Why it matters (embedded)
Model-based controllers (computed-torque, impedance, momentum observers) are only as good as the
dynamic model they use. Datasheet inertias are approximate and change when a tool or payload is
attached. Running an excitation trajectory and identifying the parameters on-device produces an
accurate, up-to-date model without a CAD teardown.

## How it works (intuition)
Command a rich *excitation trajectory* and log joint positions, velocities, accelerations, and
torques. At each sample the Recursive Newton-Euler regressor produces a matrix `Y` such that
`Y·a = τ`. Stacking many samples over-determines `a`, which is then found by **least squares**.
Only combinations the trajectory actually excites (the base parameters) are identifiable.

## Key parameters
- **Excitation trajectory** — must be persistently exciting (multi-frequency) to make `YᵀY` full-rank.
- **Base parameters** — the identifiable linear combinations; the raw inertial set is rank-deficient.
- **Signal filtering** — consistent filtering of `q̇, q̈, τ` to suppress differentiation noise.

## Reference
C. G. Atkeson, C. H. An, J. M. Hollerbach, "Estimation of Inertial Parameters of Manipulator Loads
and Links," *Int. J. Robotics Research*, 5(3), 1986.

## See also
`RecursiveNewtonEuler` (supplies the regressor), `PolynomialFitting` / `LinearRegression`
(least-squares kin), `SlotineLiAdaptiveControl` (online parameter adaptation).
