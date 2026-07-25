# Lyapunov / Sylvester Equation Solvers — Overview

## What it is
Solvers for the linear matrix equations `A X + X B = C` (Sylvester) and its symmetric relatives, the
continuous `A X + X Aᵀ = −Q` and discrete `A X Aᵀ − X = −Q` **Lyapunov** equations. The unknown is a
whole matrix `X`, not a vector.

## Why it matters (embedded)
These equations are the algebra of stability and energy. A positive-definite Lyapunov solution is a
*certificate* that a system is stable; the same equations produce the **controllability /
observability Gramians** that quantify how well a plant can be steered and seen — prerequisites for
observers, LQR, and model reduction.

## How it works (intuition)
Written out, `A X + X B = C` is a huge linear system in the entries of `X`. **Bartels-Stewart** avoids
that by first rotating `A` and `B` to (quasi-)triangular form via a Schur/eigen decomposition, which
*decouples* the unknown columns so they can be found one at a time by back-substitution — then the
result is rotated back. For symmetric `A` the eigenbasis diagonalizes everything and each entry is a
simple division by a sum of eigenvalues.

## Key parameters
- **Equation type** — Sylvester vs continuous vs discrete Lyapunov.
- **Spectrum of `A`, `B`** — determines solvability (no eigenvalues that cancel).

## Reference
R. H. Bartels, G. W. Stewart, "Solution of the Matrix Equation AX + XB = C,"
*Comm. ACM*, 15(9), 1972.

## See also
`JacobiEigenSolver`, `QrDecomposition`, `ControllabilityObservability` (Gramians),
`DiscreteAlgebraicRiccatiEquation`.
