# LU Decomposition — Overview

## What it is
A factorization `P·A = L·U` that splits a permuted matrix into a unit lower-triangular `L` and an
upper-triangular `U`. It is the general-purpose engine for dense linear systems, determinants, and
matrix inverses.

## Why it matters (embedded)
Many estimators and controllers reduce to "solve `A x = b`" for a general (non-symmetric) `A`.
Factoring `A` **once** and reusing `L`, `U` turns every subsequent solve into two cheap triangular
sweeps — a large win when the same plant matrix is solved against many right-hand sides in a loop.

## How it works (intuition)
It is Gaussian elimination with bookkeeping: each step eliminates a column below the pivot and stores
the multipliers in `L`. **Partial pivoting** swaps in the largest-magnitude pivot first, which keeps
the multipliers `≤ 1` and stops round-off from exploding. The permutation `P` records those swaps.

## Key parameters
- **Pivoting strategy** — partial pivoting (row swaps) is the stable default.
- **Reuse** — decompose once, then `Solve` repeatedly; only form `Inverse` when it is truly needed.

## Reference
Golub & Van Loan, *Matrix Computations*, Ch. 3 (Gaussian elimination with partial pivoting).

## See also
`GaussianElimination` (single-shot solve), `CholeskyDecomposition` (SPD, faster),
`QrDecomposition` (least squares).
