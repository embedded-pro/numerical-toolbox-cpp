# Total Least Squares

## Overview & Motivation

Ordinary least squares (OLS) assumes the regressors are known exactly and only the response is
noisy — it minimizes the *vertical* residuals. Real embedded systems violate that assumption:
calibrating one measured quantity against another (two drifting sensors, current vs. torque,
strain vs. force) is an **errors-in-variables** problem where *every* channel is noisy. Fitting
such data with OLS produces a systematically **biased** (attenuated) slope, a phenomenon known as
regression dilution.

Total Least Squares (TLS) removes that bias. Geometrically it minimizes the **orthogonal**
(perpendicular) distance from each point to the fitted hyperplane rather than the vertical
distance, which is the maximum-likelihood estimate when the noise is equal on all channels.

## Mathematical Theory

### The Model

Given a design matrix $A \in \mathbb{R}^{m \times n}$ and a response $b \in \mathbb{R}^{m}$, TLS
seeks the smallest perturbation $[\,\Delta A \mid \Delta b\,]$ (in Frobenius norm) that makes the
system consistent:

$$\min_{\Delta A,\ \Delta b} \bigl\|[\,\Delta A \mid \Delta b\,]\bigr\|_F
\quad\text{s.t.}\quad (A + \Delta A)\,x = b + \Delta b$$

### SVD Solution

Stack the data into the augmented matrix $M = [\,A \mid b\,] \in \mathbb{R}^{m \times (n+1)}$ and
take its singular value decomposition $M = U \Sigma V^{\top}$. The right-singular vector
$v_{n+1}$ associated with the **smallest** singular value $\sigma_{n+1}$ spans the direction of
least variance — the normal of the best-fit hyperplane. Partitioning

$$v_{n+1} = \begin{bmatrix} v_{1:n} \\ v_{n+1,\,n+1} \end{bmatrix},
\qquad x = -\,\frac{v_{1:n}}{v_{n+1,\,n+1}}$$

recovers the coefficients. The construction is exactly the Golub–Van Loan (1980) result: the
minimal perturbation is $\sigma_{n+1}\, u_{n+1} v_{n+1}^{\top}$, and $\sigma_{n+1}$ is the
orthogonal residual norm.

### Existence

The solution exists (is *generic*) only when the last entry $v_{n+1,\,n+1} \neq 0$. If it vanishes,
the smallest singular direction lies entirely in the column space of $A$ (e.g. a rank-deficient or
all-zero regressor column) and no finite coefficient vector satisfies the fit — `Fit` returns
`false`.

## Complexity Analysis

| Case    | Time                | Space            | Notes                                             |
|---------|---------------------|------------------|---------------------------------------------------|
| Best    | $O(m\,n^2)$         | $O(m\,n)$        | Golub–Kahan bidiagonalization dominates           |
| Average | $O(m\,n^2)$         | $O(m\,n)$        | Implicit-QR sweeps converge in $O(n)$ per value   |
| Worst   | $O(m\,n^2)$         | $O(m\,n)$        | Bounded iteration cap in the SVD engine           |

All storage is stack-allocated (`std::array`-backed `math::Matrix`); no heap, no recursion.

## Step-by-Step Walkthrough

Fit $b = 2a$ from four clean points $a = (1,2,3,4)$, $b = (2,4,6,8)$.

1. Augment: $M = \begin{bmatrix} 1 & 2 \\ 2 & 4 \\ 3 & 6 \\ 4 & 8 \end{bmatrix}$ — exactly rank 1.
2. SVD gives $\sigma_1 \approx 12.25$, $\sigma_2 = 0$ (the data hugs a line).
3. The smallest right-singular vector is $v_2 = \tfrac{1}{\sqrt5}(2, -1)$.
4. Partition: $x = -\,v_{2,1} / v_{2,2} = -\,\tfrac{2/\sqrt5}{-1/\sqrt5} = 2$. ✓

## Pitfalls & Edge Cases

- **Column scaling.** The SVD is dominated by the largest-norm column. Scale the columns of
  `A` and `b` to comparable magnitudes before fitting, otherwise the TLS/OLS distinction is lost.
- **Singular-value gap.** A tiny gap between the two smallest singular values signals an ill-posed
  (near-degenerate) fit; the recovered coefficients become sensitive to noise.
- **Degeneracy.** An all-zero or linearly dependent regressor column drives the last entry of the
  smallest singular vector to zero; `Fit` reports `false` rather than dividing by zero.
- **Minimal system.** Requires `Samples >= Features + 1` (enforced by `static_assert`).

## Variants & Generalizations

- **Weighted / generalized TLS** — scale rows/columns by known noise covariances before the SVD.
- **Regularized (truncated) TLS** — discard trailing singular directions for ill-conditioned data.
- **OLS counterparts** — [Linear Regression](LinearRegression.md) and
  [Polynomial Fitting](PolynomialFitting.md) solve the noise-free-regressor case.

## Applications

- **Sensor calibration** — fitting one measured quantity against another (two drifting sensors,
  current vs. torque, strain vs. force) where *both* channels carry noise.
- **System identification** — parameter estimation when the regressors themselves are measured,
  removing the OLS bias that would corrupt an identified plant model.
- **Line/plane fitting** — geometric fitting that minimizes perpendicular distance, e.g. estimating
  a boundary or feature direction from noisy point clouds.
- **Model reduction** — the smallest-singular-direction analysis flags near-degenerate fits and
  quantifies the orthogonal residual.

## Connections to Other Algorithms

- [`SingularValueDecomposition`](../solvers/SingularValueDecomposition.md) — the numerical engine;
  TLS is the SVD of `[A | b]` read from the smallest-singular-value end.
- [Linear Regression](LinearRegression.md) / [Polynomial Fitting](PolynomialFitting.md) — the
  ordinary least-squares counterparts (exact regressors, vertical residuals).
- Symmetric eigenvalue / [Jacobi eigen solver](../solvers/JacobiEigenSolver.md) — TLS on the
  augmented normal matrix `[A | b]ᵀ[A | b]` reduces to its smallest-eigenvalue eigenvector.

## References & Further Reading

- G. H. Golub, C. F. Van Loan, "An Analysis of the Total Least Squares Problem,"
  *SIAM J. Numer. Anal.*, 17(6), 1980.
- S. Van Huffel, J. Vandewalle, *The Total Least Squares Problem: Computational Aspects and
  Analysis*, SIAM, 1991.
- G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., Ch. 6 (least squares) & Ch. 8 (SVD).
