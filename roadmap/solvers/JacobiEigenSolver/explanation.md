# Symmetric Eigenvalue Solver (Jacobi) — Overview

## What it is
An iterative method that finds the eigenvalues and eigenvectors of a **symmetric** matrix by applying
a sequence of plane (Jacobi) rotations that progressively zero the off-diagonal entries until only a
diagonal of eigenvalues remains.

## Why it matters (embedded)
Symmetric eigenproblems are the heart of **PCA / feature extraction**, modal (vibration) analysis,
and covariance conditioning. On-device these matrices are small — a few states, an inertia tensor, a
covariance block — exactly the regime where Jacobi is both simplest and most accurate.

## How it works (intuition)
Each rotation targets one off-diagonal pair `(p,q)` and turns it into zero, at the cost of slightly
disturbing others — but the total off-diagonal "energy" always decreases. Sweeping over every pair
repeatedly drives the matrix to diagonal form; the accumulated rotations spell out the eigenvectors.
Convergence is quadratic, so a handful of sweeps suffices.

## Key parameters
- **Tolerance** — the off-diagonal norm at which the matrix is declared diagonal.
- **Max sweeps** — an upper bound that guarantees bounded, real-time-friendly work.

## Reference
Golub & Van Loan, *Matrix Computations*, Ch. 8 (symmetric eigenproblem / cyclic Jacobi).

## See also
`SingularValueDecomposition` (built on the same rotations), `LyapunovSylvester`,
`QrDecomposition`.
