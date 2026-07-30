# Householder Transform

## Overview & Motivation

A Householder reflector is the workhorse of numerically stable dense linear algebra. A single reflector mirrors a vector onto a coordinate axis, zeroing every entry below a chosen pivot in one orthogonal step. Chaining reflectors triangularizes a matrix (QR), bidiagonalizes it (SVD), or tridiagonalizes a symmetric matrix (eigensolvers). `HouseholderVector` computes the reflector for one sub-column; it is the shared primitive those factorizations call.

## Mathematical Theory

For a vector $x$, the Householder reflector is the orthogonal matrix

$$H = I - \beta\, v v^\top$$

chosen so that $Hx$ is zero below the pivot. With $\sigma = \sum_{i>\text{start}} x_i^2$ and $\|x\| = \sqrt{x_{\text{start}}^2 + \sigma}$, the reflector maps $x_{\text{start}} \mapsto \mp\|x\|$. The pivot sign is chosen as $-\operatorname{sign}(x_{\text{start}})\|x\|$ to avoid cancellation:

$$v_{\text{start}} = 1, \quad v_i = x_i / v_0, \quad \beta = \frac{2 v_0^2}{\sigma + v_0^2}$$

$H$ is symmetric and orthogonal ($H = H^\top = H^{-1}$), so applying it is backward stable.

## Complexity Analysis

| Operation          | Time   | Space  | Notes                                   |
|--------------------|--------|--------|-----------------------------------------|
| HouseholderVector  | $O(n)$ | $O(1)$ | Builds `v` (stored implicit unit pivot) |
| Apply $H$ to a vector | $O(n)$ | $O(1)$ | `x - β·v·(vᵀx)` — never form `H`         |

## Step-by-Step Walkthrough

For $x = [4, 3, 0, 0]^\top$, pivot `start = 0`: $\sigma = 9$, $\|x\| = 5$. Since $x_0 > 0$, $v_0 = -\sigma/(x_0 + \|x\|) = -1$, giving $\beta = 1$ and $v = [1, -3, 0, 0]^\top$. Reflecting yields $Hx = [-5, 0, 0, 0]^\top$ — the sub-column collapsed onto the axis.

## Pitfalls & Edge Cases

**Already-zero sub-column** — when $\sigma \approx 0$ there is nothing to zero; the routine returns $\beta = 0$ (identity reflector) and callers skip the update.

**Never form `H` explicitly** — apply it as `x − β·v·(vᵀx)` to keep the cost $O(n)$ per column instead of $O(n^2)$.

**Float-only** — `static_assert(std::is_floating_point_v<T>)`; the sign-fixing and normalisation assume real floating-point arithmetic.

## Variants & Generalizations

Givens rotations (`math::GivensRotation`) achieve the same zeroing one entry at a time, preferable for sparse or streaming updates. Complex Householder reflectors extend this to unitary triangularization.

## Applications

- **QR decomposition** — one reflector per column triangularizes `A`.
- **SVD / eigensolvers** — bidiagonalization and tridiagonalization.
- **Square-root Kalman filtering** — covariance factor updates.

## Connections to Other Algorithms

Used by `solvers::QrDecomposition`; complements `math::GivensRotation` and `math::SolveUpperTriangular`. Operates on `math::Vector`.

## References & Further Reading

- Householder, A. S., "Unitary Triangularization of a Nonsymmetric Matrix", JACM 5(4), 1958
- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., §5.1 (Householder reflections)
