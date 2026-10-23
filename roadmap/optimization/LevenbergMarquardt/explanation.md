# Levenberg–Marquardt — Overview

## What it is
An iterative solver for "find the state that makes these residuals as small as possible" when the
residuals depend nonlinearly on the state. Each iteration linearizes the residuals, solves a small damped
linear least-squares problem for a correction, and applies it through a problem-defined update.

## Why it matters (embedded)
Inverse kinematics, the forward kinematics of parallel robots, sensor and hand-eye calibration, and scan
matching are all small nonlinear least-squares problems solved every control cycle or at commissioning.
Writing one careful solver — bounded iterations, no heap, singularity-safe damping, a pluggable update for
angles, rotations and joint limits — replaces a hand-rolled Newton loop in every module.

## How it works (intuition)
Near the current guess the residual behaves like a linear function of a small correction; Gauss–Newton
solves that linear problem exactly. When the linearization is poor or nearly singular, a damping term
penalizes large corrections, trading speed for robustness — the Levenberg–Marquardt idea. The correction
is applied by a "retraction" supplied by the problem: ordinary addition for plain vectors, addition plus
clamping for bounded states, or a rotation update for orientations, so the state never leaves its valid
set.

## Key parameters
- **Damping policy** — constant `λ`, or a problem-aware schedule (e.g. zero away from singularities).
- **Tolerance** and **iteration budget** — stopping rule and worst-case cost.
- **Step cap** — maximum correction per iteration.

## Reference
K. Levenberg, *Quarterly of Applied Mathematics*, 2(2), 1944; D. Marquardt, *SIAM J. Appl. Math.*, 11(2),
1963; J. Nocedal, S. Wright, *Numerical Optimization*, 2nd ed. (2006), Ch. 10; P.-A. Absil, R. Mahony,
R. Sepulchre, *Optimization Algorithms on Matrix Manifolds* (2008), Ch. 4 (retractions).

## See also
`GradientDescent` / `Adam` (first-order optimizers for general costs), `LuDecomposition` (the step solve),
`DampedPseudoInverse` (#62, the same damping for one linear solve), `SE3Transform` (#53, pose retractions).
