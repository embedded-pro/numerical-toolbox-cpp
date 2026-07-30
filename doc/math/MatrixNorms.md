# Matrix & Vector Norms

## Overview & Motivation

Linear algebra computations — solvers, Kalman filter covariance updates, regression — depend on the numerical health of the matrices involved. Norms formalise the notion of "size" for matrices and vectors and are the building blocks of every conditioning, stability, and error-bound estimate in the library. They are also the cheapest such quantities to compute: a single pass over the entries with no allocation, suitable for real-time embedded paths.

Vector normalisation, closely related, produces the unit-length direction of a vector and is a recurring primitive in geometry, attitude estimation, and gradient methods.

## Mathematical Theory

### Dot Product & Vector Norm

For vectors $\mathbf{a}, \mathbf{b} \in \mathbb{R}^n$, the dot product is $\mathbf{a}\cdot\mathbf{b} = \sum_{i=1}^{n} a_i b_i$. The Euclidean (L2) norm is the square root of the self dot product:

$$\|\mathbf{v}\|_2 = \sqrt{\mathbf{v}\cdot\mathbf{v}} = \sqrt{\sum_{i=1}^{n} v_i^2}$$

`VectorNorm` is implemented in terms of `DotProduct`. The unit vector $\hat{\mathbf{v}} = \mathbf{v} / \|\mathbf{v}\|_2$ satisfies $\|\hat{\mathbf{v}}\|_2 = 1$. Normalisation is undefined when $\|\mathbf{v}\|_2 = 0$ and must be guarded.

### Matrix Norms

**Frobenius norm** — treats the matrix as a flattened vector:

$$\|A\|_F = \sqrt{\sum_{i=1}^{m}\sum_{j=1}^{n} a_{ij}^2}$$

It is rotationally invariant under unitary transformations and cheap to compute.

**1-norm (maximum absolute column sum)**:

$$\|A\|_1 = \max_{1 \le j \le n} \sum_{i=1}^{m} |a_{ij}|$$

**Infinity norm (maximum absolute row sum)**:

$$\|A\|_\infty = \max_{1 \le i \le m} \sum_{j=1}^{n} |a_{ij}|$$

The 1-norm and infinity-norm are dual: $\|A\|_\infty = \|A^\top\|_1$.

## Complexity Analysis

| Operation     | Time    | Space  | Notes                                  |
|---------------|---------|--------|----------------------------------------|
| DotProduct    | $O(n)$  | $O(1)$ | Single pass; `VectorNorm` builds on it |
| FrobeniusNorm | $O(mn)$ | $O(1)$ | Single pass, no allocation   |
| OneNorm       | $O(mn)$ | $O(1)$ | Column-wise sum, running max |
| InfinityNorm  | $O(mn)$ | $O(1)$ | Row-wise sum, running max    |
| VectorNorm    | $O(n)$  | $O(1)$ | Single pass                  |
| Normalize     | $O(n)$  | $O(n)$ | Output vector on stack       |

## Step-by-Step Walkthrough

Matrix $A = \begin{bmatrix}3 & 1 \\ 1 & 2\end{bmatrix}$:

1. **FrobeniusNorm**: $\sqrt{9 + 1 + 1 + 4} = \sqrt{15} \approx 3.873$
2. **OneNorm**: column 0 sum $= |3| + |1| = 4$; column 1 sum $= |1| + |2| = 3$; max $= 4$
3. **InfinityNorm**: row 0 sum $= |3| + |1| = 4$; row 1 sum $= |1| + |2| = 3$; max $= 4$

Vector $\mathbf{v} = [3,\, 4]^\top$: $\|\mathbf{v}\|_2 = 5$, and $\hat{\mathbf{v}} = [0.6,\, 0.8]^\top$.

## Pitfalls & Edge Cases

**Zero vector normalisation** — dividing by $\|\mathbf{v}\|_2 = 0$ is undefined. The implementation returns an empty optional for near-zero norms.

**Fast-math semantics** — `#pragma GCC optimize("fast-math")` may reorder floating-point operations. The norms are sums of non-negative values, so reordering does not change the sign of the result, but catastrophic cancellation can still occur for near-zero off-diagonal entries.

**Non-square matrices** — FrobeniusNorm, OneNorm, and InfinityNorm apply to any $m \times n$ matrix.

## Variants & Generalizations

The **spectral norm** (largest singular value, $\|A\|_2$) is the tightest but requires an SVD — $O(N^3)$ with a large constant, unsuitable for embedded real-time paths. The 1-norm and infinity-norm are cheap upper bounds used throughout the library instead.

General **p-norms** and weighted norms generalise the vector case; the L2 norm is the only one currently exposed because it is the natural quantity for geometric and least-squares work.

## Applications

- **Conditioning estimates**: the 1-norm feeds the condition number (see `solvers::ConditionNumber`).
- **Convergence tests**: iterative solvers and optimisers stop when a residual norm falls below tolerance.
- **Attitude / geometry**: vector normalisation produces unit direction and rotation axes.
- **Covariance sanity**: the Frobenius norm of a covariance matrix bounds its total variance.

## Connections to Other Algorithms

The 1-norm is the norm used by `solvers::ConditionNumber` for its $\|A\|\cdot\|A^{-1}\|$ estimate. The `math::Matrix` type provides the storage and transpose the norms operate on. Norm-based residual tests appear in `solvers` and `optimization`.

## References & Further Reading

- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., Chapter 2 (matrix norms)
- Trefethen, L. N. & Bau, D., "Numerical Linear Algebra", Lecture 3 (norms)
- Higham, N. J., "Accuracy and Stability of Numerical Algorithms", 2nd ed.
