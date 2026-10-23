# Dense Convex QP (Goldfarb–Idnani) — Overview

## What it is
A solver for problems of the form "minimize a convex quadratic cost subject to linear equalities and
inequalities" — the workhorse of constrained control and allocation. It handles small, dense problems
exactly, in a bounded number of steps, without dynamic memory.

## Why it matters (embedded)
Many embedded decisions are QPs: distribute a wrench over cables that can only pull, allocate thrust to
motors that saturate, track several tasks with priorities and joint limits, keep MPC inputs in bounds.
Heuristics such as clipping or a fixed number of projection sweeps are cheap but can be wrong at the
edges; an exact, finite active-set method gives the true optimum with a predictable worst case.

## How it works (intuition)
Start from the unconstrained minimum, which is cheap thanks to a Cholesky factor of the cost. If it
violates a constraint, pull the solution towards that constraint along the direction that hurts the cost
least, raising its multiplier. If, on the way, an already active constraint's multiplier would turn
negative (it is pushing the wrong way), release it first. Repeat until nothing is violated. Because every
intermediate point is optimal for the constraints chosen so far, the method never needs a feasible start,
and a running orthogonal factorization keeps each step at quadratic cost.

## Key parameters
- **Hessian `H`** — must be positive definite (strictly convex cost).
- **Constraint rows** — equalities are always enforced; inequalities only when they bind.
- **Iteration cap** and **feasibility tolerance** — worst-case cost and acceptance threshold.

## Reference
D. Goldfarb, A. Idnani, "A numerically stable dual method for solving strictly convex quadratic
programs," *Mathematical Programming*, 27(1), 1983; M. J. D. Powell, "On the quadratic programming
algorithm of Goldfarb and Idnani," *Mathematical Programming Study*, 25, 1985.

## See also
`CholeskyDecomposition`, `GivensRotation`, `Mpc` (box-constrained inputs), `DampedPseudoInverse` (#62,
the unconstrained counterpart); robotics-toolbox-cpp's cable-tension distribution (exact path).
