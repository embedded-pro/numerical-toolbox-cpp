# Singular Value Decomposition

## Overview & Motivation

The singular value decomposition (SVD) factors any $m \times n$ matrix $A$ into
two orthonormal rotation factors and a non-negative diagonal, revealing the fundamental geometric
action of $A$ as a stretch along $k = \min(m, n)$ orthogonal directions. It is the most numerically stable
matrix factorization available and exposes rank, condition number, and the pseudo-inverse directly
from its output. In embedded systems, SVD powers robust least-squares solvers for sensor calibration,
redundant-actuator control allocation, and low-rank model reduction — all applications where the
matrix $A$ may be ill-conditioned or rank-deficient and normal-equation approaches fail.

## Mathematical Theory

### Factorization

For $A \in \mathbb{R}^{m \times n}$ and $k = \min(m, n)$, the thin SVD is

$$A = U \Sigma V^\top,$$

where $U \in \mathbb{R}^{m \times k}$ and $V \in \mathbb{R}^{n \times k}$ have orthonormal columns
($U^\top U = V^\top V = I_k$) and $\Sigma = \mathrm{diag}(\sigma_1, \ldots, \sigma_k)$ with
$\sigma_1 \geq \cdots \geq \sigma_k \geq 0$. For a tall or square matrix ($m \geq n$) $V$ is square
and orthogonal; for a wide matrix ($m < n$) $U$ is.
The scalars $\sigma_i$ are the **singular values** of $A$; the columns of $U$ and $V$ are
the left and right **singular vectors**.

### Wide Matrices

The Golub–Kahan bidiagonalization below runs on a tall matrix. A wide $A$ is handled through its
transpose: factor $A^\top = U' \Sigma V'^\top$ (tall), then $A = V' \Sigma U'^\top$, so $U = V'$ and
$V = U'$. The singular values are shared, and no extra storage is needed beyond the tall factors.

### Connection to the Symmetric Eigenproblem

The singular values satisfy $\sigma_i^2 = \lambda_i(A^\top A)$, where $\lambda_i$ denotes the
$i$-th eigenvalue of $A^\top A$. This ties the SVD to the symmetric eigenproblem and provides
a cross-check: computing the eigenvalues of $A^\top A$ via the Jacobi or tridiagonal-QR solver
should yield the squared singular values.

### Pseudo-Inverse

The Moore-Penrose pseudo-inverse is

$$A^+ = V \Sigma^+ U^\top, \quad \Sigma^+_{ii} = \begin{cases} 1/\sigma_i & \sigma_i > \tau \\ 0 & \text{otherwise} \end{cases}$$

for a threshold $\tau > 0$. Truncating small singular values regularises the solution against
near-null-space components, avoiding division by near-zero values that arise from rank-deficiency
or numerical noise.

### Rank and Condition Number

$$\mathrm{rank}(A) = \#\{i : \sigma_i > \tau\}, \quad \kappa(A) = \sigma_1 / \sigma_k.$$

The condition number $\kappa$ measures sensitivity: a system with $\kappa \gg 1$ amplifies
input perturbations by a factor of $\kappa$ in the least-squares solution.

## Complexity Analysis

| Phase             | Time                                            | Space    | Notes                                        |
|-------------------|-------------------------------------------------|----------|----------------------------------------------|
| Bidiagonalization | $O(Mk^2 - k^3/3)$                               | $O(mn)$  | Two-sided Householder reflectors             |
| QR sweeps         | $O(k^2)$ per iteration, $O(k)$ total iterations | $O(k^2)$ | Golub-Reinsch implicit-shift; converges fast |
| Pseudo-inverse    | $O(k\,mn)$                                      | $O(nm)$  | Matrix triple product $V \Sigma^+ U^\top$    |
| Total             | $O(Mk^2 + k^3)$                                 | $O(mn)$  | Dominated by bidiagonalization               |

Here $M = \max(m, n)$ and $k = \min(m, n)$: a wide matrix costs the same as its transpose.

All storage is stack-allocated; no heap is used. The bidiagonal form is stored implicitly in
the working copy of $A$ alongside the Householder reflector scalars.

## Step-by-Step Walkthrough

**Input:** $A = \begin{pmatrix} 3 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & 2 \end{pmatrix}$ (diagonal matrix).

**Phase 1 — Bidiagonalization:** $A$ is already diagonal (and hence upper-bidiagonal). All
Householder reflectors are trivial ($\beta = 0$). The bidiagonal form retains $B = A$;
$U = I_3$, $V = I_3$.

**Phase 2 — QR sweeps:** The superdiagonal is already zero, so the sweep loop exits immediately.

**Sign correction and sort:** $\sigma = (3, 1, 2)$ after reading the diagonal; the entry
$-1$ has a negative sign, so it is flipped to $+1$ and the corresponding column of $U$ is
negated. After descending sort: $\sigma = (3, 2, 1)$.

## Pitfalls & Edge Cases

- **Rank deficiency:** a singular value of exactly zero is harmless; the pseudo-inverse
  threshold $\tau$ must be chosen relative to $\sigma_1$ and machine epsilon to avoid
  false zero-detection.
- **Ill-conditioning in the shift:** the implicit shift $\mu$ is derived from the trailing
  $2 \times 2$ submatrix; a zero superdiagonal entry deflates the problem before the shift
  is computed, preventing division by zero.
- **Sign ambiguity:** singular vectors are defined up to sign; the implementation folds negative
  diagonal entries into the sign of the corresponding $U$ column so that all $\sigma_i \geq 0$.
- **Repeated singular values:** the corresponding singular-vector subspaces are correct but the
  individual vectors are not unique; tests should check subspace properties, not individual vectors.
- **Condition number with zero $\sigma_{\min}$:** a rank-deficient matrix has $\sigma_k = 0$;
  `ConditionNumber()` returns zero as a sentinel in this case rather than dividing by zero.

## Variants & Generalizations

- **Full SVD:** $U \in \mathbb{R}^{m \times m}$ and $V \in \mathbb{R}^{n \times n}$; the extra columns
  span the left and right null spaces of $A$. The thin SVD suffices for least squares; a null-space
  projector $I - V V^\top$ needs only the thin $V$.
- **Truncated SVD:** retain only the $k$ largest singular values for low-rank approximation
  $A \approx U_k \Sigma_k V_k^\top$; the approximation error in Frobenius norm equals
  $\sqrt{\sigma_{k+1}^2 + \cdots + \sigma_n^2}$ (Eckart-Young theorem).
- **Divide-and-conquer SVD:** splits the bidiagonal matrix recursively; $O(n^2)$ per level
  and $O(n \log n)$ overall, faster than Golub-Reinsch for large $n$ but incompatible with
  a no-recursion embedded requirement.
- **Jacobi SVD:** applies one-sided plane rotations directly to $A$; simpler but $O(n^3)$
  per sweep with slower convergence than implicit-shift QR.

## Applications

- **Least-squares regression:** $x = A^+ b$ minimises $\|Ax - b\|_2$ and is unique among
  minimum-norm solutions when $A$ is rank-deficient. For a wide, full-row-rank $A$ (an
  underdetermined system) it is the minimum-norm exact solution $A^\top (A A^\top)^{-1} b$.
- **Control allocation:** maps actuator space to output space; the pseudo-inverse distributes
  effort across redundant actuators while respecting the null space.
- **System identification:** SVD of a Hankel matrix (Ho-Kalman) recovers system order (rank)
  and state-space realization from impulse-response data.
- **Principal-component analysis (offline):** left singular vectors of the centred data matrix
  are the principal components; singular values are the standard deviations along each axis.
- **Condition monitoring:** a rising condition number $\kappa$ before solving warns that the
  system is near-singular and the solution may be unreliable.

## Connections to Other Algorithms

- `QrDecomposition` provides the Householder reflectors reused in the bidiagonalization phase.
- `JacobiEigenSolver` (symmetric eigenproblem) is an alternative route to the same result via
  $\sigma_i^2 = \lambda_i(A^\top A)$; the two cross-check each other.
- `LuDecomposition` and `GaussianElimination` solve square systems but cannot handle
  rank-deficient or overdetermined cases; SVD supersedes them for those inputs.
- `TotalLeastSquares` (item 44) builds directly on SVD: the solution lies in the right singular
  vector corresponding to the smallest singular value of the augmented matrix $[A\ b]$.

## References & Further Reading

- G. Golub and W. Kahan, "Calculating the Singular Values and Pseudo-Inverse of a Matrix," *SIAM J. Numer. Anal.*, 2(2), 1965.
- G. Golub and C. Reinsch, "Singular Value Decomposition and Least Squares Solutions," *Numer. Math.*, 14(5), 1970.
- G. H. Golub and C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins UP, 2013, Ch. 6 & 8.
- W. H. Press et al., *Numerical Recipes in C*, 3rd ed., Cambridge UP, 2007, §2.6.
