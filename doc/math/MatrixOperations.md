# Matrix Operations

## Overview & Motivation

Small structural matrix utilities that are needed across the library but do not belong to any single algorithm:

- **`Symmetrize`** enforces exact symmetry on a matrix that should be symmetric in theory but drifts under floating-point round-off.
- **`CongruenceTransform`** computes the quadratic form `A·M·Aᵀ` — the single most common shape in covariance-propagating code (Kalman predict `F·P·Fᵀ`, innovation covariance `H·P·Hᵀ`, Joseph update `(I−KH)·P·(I−KH)ᵀ`, etc.).

- **`Diagonal`** and **`DiagonalOf`** convert between a vector and a diagonal matrix.
- **`ScaleColumns`** and **`DiagonalCongruence`** compute `M·diag(d)` and `diag(d)·M·diag(d)` without forming the diagonal matrix: per-joint gains, sign conventions and unit scalings applied to Jacobians and mass matrices.

The first two are recurring needs in Kalman filters, EM parameter updates, and Riccati/Lyapunov solvers; the diagonal products recur wherever a matrix is expressed in rescaled coordinates.

## Mathematical Theory

Any square matrix decomposes into a symmetric and a skew-symmetric part:

$$M = \underbrace{\tfrac{1}{2}(M + M^\top)}_{\text{symmetric}} + \underbrace{\tfrac{1}{2}(M - M^\top)}_{\text{skew}}$$

`Symmetrize` returns the symmetric part $\tfrac{1}{2}(M + M^\top)$. It is the orthogonal projection (in the Frobenius inner product) of $M$ onto the subspace of symmetric matrices, so it is the *closest* symmetric matrix to $M$. Applying it to an already-symmetric matrix is a no-op (idempotent).

If $M$ is symmetric, `CongruenceTransform` returns a symmetric result exactly (in exact arithmetic): $(AMA^\top)^\top = A M^\top A^\top = A M A^\top$. This makes it the natural building block for propagating a covariance $P$ through a linear map $A$: $P \mapsto A P A^\top$.

With $D = \operatorname{diag}(d)$, a product with a diagonal matrix scales columns, or rows and columns, entry by entry:

$$(M D)_{ij} = M_{ij}\, d_j, \qquad (D M D)_{ij} = d_i\, M_{ij}\, d_j$$

so neither product needs a matrix multiplication. $D M D$ is a congruence: it keeps a symmetric $M$ symmetric, and a positive-definite $M$ positive definite when every $d_i \neq 0$.

## Complexity Analysis

| Operation            | Time               | Space    | Notes                                                                   |
|----------------------|--------------------|----------|-------------------------------------------------------------------------|
| Symmetrize           | $O(n^2)$           | $O(n^2)$ | One transpose, add, scale                                               |
| CongruenceTransform  | $O(n^2 m + n m^2)$ | $O(nm)$  | For $A \in \mathbb{R}^{n\times m}$, two matrix products                 |
| Diagonal, DiagonalOf | $O(n)$             | $O(n^2)$ | Copies the diagonal only                                                |
| ScaleColumns         | $O(nm)$            | $O(nm)$  | One multiplication per entry                                            |
| DiagonalCongruence   | $O(n^2)$           | $O(n^2)$ | Two multiplications per entry, against $O(n^3)$ for two matrix products |

## Step-by-Step Walkthrough

**Symmetrize** — for $M = \begin{bmatrix}1 & 3 \\ -1 & 2\end{bmatrix}$: $M^\top = \begin{bmatrix}1 & -1 \\ 3 & 2\end{bmatrix}$, so $\tfrac{1}{2}(M + M^\top) = \begin{bmatrix}1 & 1 \\ 1 & 2\end{bmatrix}$ — off-diagonals averaged, diagonal unchanged.

**CongruenceTransform** — with $A \in \mathbb{R}^{n\times m}$ and symmetric $M \in \mathbb{R}^{m\times m}$, the result $A M A^\top \in \mathbb{R}^{n\times n}$ is the covariance of $A x$ when $x$ has covariance $M$.

**DiagonalCongruence** — for $d = (2, -3)$ and $M = \begin{bmatrix}1 & 3 \\ -1 & 2\end{bmatrix}$: $(DMD)_{01} = 2 \cdot 3 \cdot (-3) = -18$, $(DMD)_{10} = -3 \cdot (-1) \cdot 2 = 6$ and $(DMD)_{11} = 9 \cdot 2 = 18$, as $\operatorname{diag}(d)\,M\,\operatorname{diag}(d)$ gives with two full products.

## Pitfalls & Edge Cases

**Symmetrize is not a fix for indefiniteness** — it removes the skew part but does not make a matrix positive-definite; covariance code typically also adds a small diagonal jitter (`+ εI`) separately.

**CongruenceTransform association** — the implementation evaluates `(A·M)·Aᵀ`, matching the left-associative `operator*`; results are bit-identical to hand-written `A * M * A.Transpose()`. Under `fast-math` the symmetry of the output can still carry tiny round-off asymmetry — follow with `Symmetrize` when exact symmetry is required.

**Float-only** — both are `static_assert(std::is_floating_point_v<T>)`.

## Variants & Generalizations

The skew-symmetric part $\tfrac{1}{2}(M - M^\top)$ is `Symmetrize`'s companion. The transposed congruence $A^\top M A$ (used by `DiscreteAlgebraicRiccatiEquation`) is obtained by passing `A.Transpose()`.

## Applications

- **Kalman predict / update** — `F·P·Fᵀ`, `H·P·Hᵀ`, Joseph form `(I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ` across the KF/EKF/UKF/smoother family.
- **EM / covariance updates** — re-symmetrize `Q`, `R`, `P` after asymmetric matrix products (`estimators::ExpectationMaximization`).
- **Riccati / Lyapunov solutions** — propagate and enforce symmetry of the solution matrix.

## Connections to Other Algorithms

Operate on `math::Matrix` / `math::SquareMatrix`. Consumed by the `filters::active` Kalman family and `estimators::ExpectationMaximization`; `CongruenceTransform` pairs naturally with `Symmetrize` for covariance-positivity hygiene.

## Zero-dimension Vectors

Matrix and vector dimensions must be strictly positive; a zero-length parameter vector is not
representable. Element access on an empty fixed-size array has no valid index, so allowing it would
trade a compile-time error for silent undefined behaviour.

A component without trainable parameters (for example a pooling or pure activation stage) should
report the absence of parameters explicitly — an optional parameter vector that is empty, or an
interface that has no parameter accessor at all — rather than returning a zero-length vector.

## References & Further Reading

- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., §2 (symmetric/skew decomposition)
- Higham, N. J., "Accuracy and Stability of Numerical Algorithms", 2nd ed. (symmetry enforcement in covariance recursions)
