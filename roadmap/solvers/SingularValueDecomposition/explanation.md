# Singular Value Decomposition — Overview

## What it is
The factorization `A = U Σ Vᵀ` of *any* matrix into two orthonormal rotations (`U`, `V`) and a
non-negative diagonal of **singular values** `Σ`. It is the most informative and most numerically
stable matrix factorization there is.

## Why it matters (embedded)
The SVD is the swiss-army knife of linear algebra: the **pseudo-inverse** for robust least squares
and control allocation, **rank** and **condition-number** estimation to detect ill-posed problems,
and low-rank model reduction. It underpins calibration (`TotalLeastSquares`) and redundant-actuator
mapping.

## How it works (intuition)
Two phases. First, alternating Householder reflectors squeeze `A` into a compact **bidiagonal** form
without changing its singular values. Second, implicit-shift QR sweeps apply tiny Givens rotations
that nibble the remaining off-diagonal to zero, leaving the singular values on the diagonal. The
accumulated rotations become `U` and `V`. Squaring the singular values recovers the eigenvalues of
`AᵀA`, tying the SVD directly to the symmetric eigenproblem.

## Key parameters
- **Rank tolerance** — the threshold below which a singular value is treated as zero.
- **Shape (thin vs full)** — thin factors suffice for least squares and the pseudo-inverse.

## Reference
G. Golub, W. Kahan, "Calculating the Singular Values and Pseudo-Inverse of a Matrix,"
*SIAM J. Numer. Anal.*, 2(2), 1965; Golub & Reinsch, 1970.

## See also
`QrDecomposition` and `JacobiEigenSolver` (its building blocks), `TotalLeastSquares`,
`LuDecomposition`.
