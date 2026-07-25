# Controllability / Observability Matrices & Gramians — Overview

## What it is
Two structural tests for a state-space model `(A, B, C)`. **Controllability** asks whether the inputs
can steer the state to any target; **observability** asks whether the outputs reveal every internal
state. Each reduces to the rank of a stacked matrix, and each has an energy-weighted companion — the
**Gramian** — that measures *how strongly* each direction is reachable or observable.

## Why it matters (embedded)
State-feedback control (LQR, pole placement) and state estimation (Luenberger observer, Kalman filter)
silently assume the plant is controllable and observable. If it is not, the design math produces gains
that look fine but never work on hardware. Running these checks **at design time** is a cheap insurance
policy before committing an observer or controller to a resource-constrained target.

## How it works (intuition)
Stack the reachable directions `B, AB, A²B, …` (the *Krylov* sequence). If those `n·m` columns span the
whole `n`-dimensional state space, every state is reachable — the pair is controllable. Observability is
the mirror image: stack `C, CA, CA², …` and check the same rank. The two properties are **duals** — the
observability of `(A, C)` is exactly the controllability of `(Aᵀ, Cᵀ)`, so one routine covers both. The
Gramians go further: solving a discrete Lyapunov equation gives a positive-definite matrix whose small
eigenvalues flag directions that are only *weakly* controllable/observable — the near-misses a binary
rank test hides.

## Key parameters
- **plant (A, B, C)** — the discrete state-space model under test.
- **rank tolerance** — relative threshold that decides which pivots count as non-zero; the whole result
  hinges on it near rank deficiency.
- **stability of A** — the Gramians only exist (the infinite sum converges) when `A` is Schur-stable.

## Reference
R. E. Kalman, "On the General Theory of Control Systems" / canonical structure (1960);
P. Antsaklis, A. Michel, *A Linear Systems Primer*, Birkhäuser, 2007.

## See also
`LuenbergerObserver` and `Lqr` (consumers that require observability / controllability);
`LyapunovSylvester` (item 31, solves the Gramian equations); `TransferFunctionStateSpace`
(minimality = controllable **and** observable).
