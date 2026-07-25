# QR Decomposition — Overview

## What it is
A factorization `A = Q·R` where `Q` has orthonormal columns and `R` is upper triangular. It is the
numerically robust way to solve least-squares problems and the structural backbone of the eigenvalue,
SVD, and square-root filtering algorithms.

## Why it matters (embedded)
Fitting an overdetermined model (more measurements than unknowns) is everywhere — calibration,
sensor fusion, system identification. Solving it through `QR` avoids forming `AᵀA`, whose condition
number is the *square* of `A`'s, so it preserves precision on reduced-word-length hardware.

## How it works (intuition)
A **Householder reflector** mirrors a whole column onto a coordinate axis, zeroing everything below
the diagonal in one shot; applying `Cols` reflectors triangularizes `A`, and the accumulated
reflections form the orthogonal `Q`. **Givens rotations** achieve the same by zeroing one entry at a
time, which is ideal when a single new row streams in and only a rank-1 update is needed.

## Key parameters
- **Shape (`Rows ≥ Cols`)** — tall matrices for least squares; square for general use.
- **Reflector vs rotation** — Householder for dense factorization, Givens for streaming updates.

## Reference
A. S. Householder, "Unitary Triangularization of a Nonsymmetric Matrix," *JACM*, 5(4), 1958;
Golub & Van Loan, *Matrix Computations*, Ch. 5.

## See also
`LuDecomposition` (general solve), `CholeskyDecomposition` (SPD), `SingularValueDecomposition`,
`JacobiEigenSolver`.
