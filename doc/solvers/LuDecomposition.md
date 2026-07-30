# LU Decomposition with Partial Pivoting

## Overview & Motivation

Many embedded control and estimation algorithms reduce to solving a dense linear system
$A x = b$ for a general (non-symmetric) matrix $A$. LU decomposition with partial pivoting
factors $A$ as $P A = L U$, where $P$ is a permutation, $L$ is unit lower-triangular, and
$U$ is upper-triangular. Factoring $A$ once and reusing $L$ and $U$ turns every subsequent
solve into two cheap triangular sweeps — a critical advantage in loops where the same plant
or observer matrix must be inverted against many right-hand sides at different time steps.

## Mathematical Theory

### Definitions

For an $n \times n$ matrix $A$, partial-pivoting LU factorization produces

$$P A = L U,$$

where:
- $P \in \{0,1\}^{n \times n}$ is a permutation matrix encoding row swaps,
- $L \in \mathbb{R}^{n \times n}$ is unit lower-triangular ($L_{ii} = 1$, $L_{ij} = 0$ for $j > i$),
- $U \in \mathbb{R}^{n \times n}$ is upper-triangular ($U_{ij} = 0$ for $i > j$).

### Gaussian Elimination with Partial Pivoting (GEPP)

At step $k$ of the outer loop ($k = 0, \ldots, n-1$):

1. **Pivot selection**: find $p = \arg\max_{i \geq k} |A_{ik}|$ (the largest-magnitude entry
   in the current column from the diagonal down).
2. **Row swap**: interchange rows $k$ and $p$ in the working matrix and record the swap in $P$.
3. **Elimination**: for each $i > k$ compute the multiplier $\ell_{ik} = A_{ik} / A_{kk}$,
   store it below the diagonal, and update the trailing submatrix:

$$A_{ij} \leftarrow A_{ij} - \ell_{ik} A_{kj}, \quad j > k.$$

After $n - 1$ steps the working matrix holds $U$ on and above the diagonal and the multipliers
$\ell_{ik}$ below it, packed together in a single $n \times n$ array.

### Triangular Solves

Given $P A = L U$ and a right-hand side $b$, the system $A x = b$ becomes $L U x = P b$.
Solving in two passes:

$$L y = P b \quad \text{(forward substitution)},$$
$$U x = y \quad \text{(back substitution)}.$$

Both passes cost $O(n^2)$ operations, and both reuse the shared
`math::SolveUnitLowerTriangular` / `math::SolveUpperTriangular` kernels
(from [`math/TriangularSolve.hpp`](../../numerical/math/TriangularSolve.hpp))
applied directly to the packed factor — the same triangular-solve building blocks
that back `QrDecomposition`.

The factored form is exposed for reuse by other components: `L()`, `U()`, and `P()`
return the individual factors (so callers can verify $P A = L U$ or build derived
quantities), and `IsSingular()` reports whether the last `Decompose` aborted.

### Determinant

The determinant of $A$ equals the product of $U$'s diagonal entries multiplied by the
sign of $P$:

$$\det(A) = \operatorname{sign}(P) \prod_{k=0}^{n-1} U_{kk}.$$

The sign flips each time rows are swapped; it starts at $+1$ and is multiplied by $-1$ per swap.

### Inverse

The inverse is formed by solving $A X = I$ column by column — $n$ back-and-forward
substitution pairs — at a total cost of $O(n^3)$.

## Complexity Analysis

| Operation     | Time                              | Space    | Notes                                          |
|---------------|-----------------------------------|----------|------------------------------------------------|
| `Decompose`   | $O\!\left(\tfrac{2}{3}n^3\right)$ | $O(n^2)$ | GEPP in-place; multipliers packed with $U$     |
| `Solve`       | $O(n^2)$                          | $O(n)$   | Two triangular sweeps; reuses stored factors   |
| `Determinant` | $O(n)$                            | $O(1)$   | One pass over $U$'s diagonal                   |
| `Inverse`     | $O(n^3)$                          | $O(n^2)$ | $n$ column solves; avoid when `Solve` suffices |

All storage is stack-allocated; the packed LU matrix and pivot vector are bounded by $N$.

## Step-by-Step Walkthrough

Factor $A = \begin{pmatrix} 2 & 1 & -1 \\ -3 & -1 & 2 \\ -2 & 1 & 2 \end{pmatrix}$.

**Step $k = 0$:** Largest magnitude in column 0 is $|-3| = 3$ at row 1. Swap rows 0 and 1.
Working matrix: $\begin{pmatrix} -3 & -1 & 2 \\ 2 & 1 & -1 \\ -2 & 1 & 2 \end{pmatrix}$.
Multipliers: $\ell_{10} = 2/(-3) = -2/3$, $\ell_{20} = -2/(-3) = 2/3$.
After elimination: $U_{0\cdot} = (-3,-1,2)$; rows 1 and 2 updated.

**Step $k = 1$:** Select pivot in column 1 from rows 1 onward. Possibly swap. Compute
multiplier $\ell_{21}$ and update $U_{2\cdot}$.

**Step $k = 2$:** Single diagonal entry remains; $U_{22}$ is read directly.

The resulting $L$ has the stored multipliers below the diagonal (with unit diagonal), and $U$
sits on and above it.

## Pitfalls & Edge Cases

- **Zero pivot after partial pivoting**: if the largest candidate in a column is below the
  singularity threshold $\varepsilon$, the matrix is (numerically) rank-deficient. The
  factorization aborts and returns `false`; the caller must not invoke `Solve` or `Inverse`.
- **Skipping pivoting**: without row swaps, a matrix with $A_{00} = 0$ would cause immediate
  division by zero; partial pivoting is mandatory for general matrices.
- **Growth factor**: partial pivoting bounds the element growth factor at $2^{n-1}$; in
  practice growth is far smaller. For adversarial matrices, complete pivoting or iterative
  refinement provides stronger guarantees.
- **Reusing the factored form**: `Inverse` calls `Solve` $n$ times and is $O(n^3)$; prefer
  `Solve` for a single right-hand side to avoid the extra $O(n^3)$ work and the conditioning
  penalty of computing the inverse explicitly.
- **Symmetric positive-definite systems**: use `CholeskyDecomposition` instead — it is
  roughly twice as fast and requires no pivoting.

## Variants & Generalizations

- **Complete pivoting (GECP)**: swaps both rows and columns to achieve a smaller growth
  factor bound; rarely needed in practice but theoretically more stable.
- **Block LU**: partitions $A$ into blocks for cache-friendly BLAS-3 operations on large
  matrices; impractical for the small $n$ typical in embedded applications.
- **Sparse LU (SuperLU / KLU)**: reorders columns to minimise fill-in; relevant for
  finite-element or circuit matrices, not for dense embedded systems.
- **Recursive LU**: divides the matrix recursively for parallelism; incompatible with the
  no-recursion constraint of this library.

## Applications

- General dense linear system solvers for observer and controller gains.
- One-shot or repeated-solve scenarios in model-predictive control warm-starting.
- Determinant computation for stability margins and matrix rank tests.
- Matrix inversion for covariance update steps where the explicit inverse is required.

## Connections to Other Algorithms

- `GaussianElimination` performs the same GEPP steps but does not retain the factored form,
  making it a single-shot solver; `LuDecomposition` amortises the $O(n^3)$ cost over
  multiple `Solve` calls.
- `CholeskyDecomposition` is a specialised $O\!\left(\tfrac{1}{3}n^3\right)$ factorization
  for symmetric positive-definite matrices — roughly $2\times$ faster and without the
  overhead of pivoting.
- `QrDecomposition` is preferred for least-squares (overdetermined) problems; for square
  full-rank systems, LU is faster.
- `ConditionNumber` can be estimated cheaply after factorization by examining the ratio
  $\max_k |U_{kk}| / \min_k |U_{kk}|$.

## References & Further Reading

- G. H. Golub & C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins UP, 2013, Ch. 3.
- W. H. Press et al., *Numerical Recipes in C*, 3rd ed., Cambridge UP, 2007, §2.3.
- L. N. Trefethen & D. Bau, *Numerical Linear Algebra*, SIAM, 1997, Lecture 20–22.
