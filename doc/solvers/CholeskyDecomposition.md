# Cholesky Decomposition

## Overview & Motivation

The **Cholesky decomposition** factors a symmetric positive-definite matrix $A$ into the product $A = LL^T$, where $L$ is a lower triangular matrix with positive diagonal entries. This is the matrix equivalent of taking a "square root."

It is approximately twice as fast as LU decomposition for symmetric positive-definite systems and is numerically stable without pivoting. In this library, it is used primarily by the Unscented Kalman Filter to generate sigma points from the state covariance matrix.

## Mathematical Theory

Given $A \in \mathbb{R}^{n \times n}$ symmetric positive-definite, compute $L$ such that $A = LL^T$:

$$L_{jj} = \sqrt{A_{jj} - \sum_{k=0}^{j-1} L_{jk}^2}$$

$$L_{ij} = \frac{1}{L_{jj}} \left( A_{ij} - \sum_{k=0}^{j-1} L_{ik} L_{jk} \right), \quad i > j$$

## Complexity Analysis

| Operation     | Time       | Space    | Notes                                        |
|---------------|------------|----------|----------------------------------------------|
| Decomposition | $O(n^3/6)$ | $O(n^2)$ | Half the cost of general LU decomposition   |

## Step-by-Step Walkthrough

**Input:** $A = \begin{bmatrix} 4 & 2 \\ 2 & 5 \end{bmatrix}$ (symmetric positive-definite).

Compute $L$ column by column:

| Step | Formula | Value |
|------|---------|-------|
| $L_{00}$ | $\sqrt{A_{00}} = \sqrt{4}$ | $2$ |
| $L_{10}$ | $A_{10} / L_{00} = 2 / 2$ | $1$ |
| $L_{11}$ | $\sqrt{A_{11} - L_{10}^2} = \sqrt{5 - 1}$ | $2$ |

**Result:**

$$L = \begin{bmatrix} 2 & 0 \\ 1 & 2 \end{bmatrix}$$

**Verify:** $LL^T = \begin{bmatrix} 2 & 0 \\ 1 & 2 \end{bmatrix} \begin{bmatrix} 2 & 1 \\ 0 & 2 \end{bmatrix} = \begin{bmatrix} 4 & 2 \\ 2 & 5 \end{bmatrix} = A$ ✓

## Pitfalls & Edge Cases

- **Non-positive-definite input.** If $A$ is not positive-definite, a diagonal element under the square root will be negative, producing NaN. Always verify the input or add a small regularization $A + \epsilon I$.
- **Numerical precision.** In fixed-point arithmetic, the intermediate sums and square root can lose precision. Use float for the decomposition and convert back if needed.
- **Zero diagonal.** A zero or near-zero diagonal in $L$ indicates a singular or nearly singular matrix.

## Variants & Generalizations

- **$LDL^T$ Decomposition.** Avoids the square root by factoring $A = LDL^T$ where $D$ is diagonal. Preferred in fixed-point arithmetic where `sqrt` is expensive or imprecise.
- **Cholesky–Banachiewicz vs Cholesky–Crout.** Two orderings of the same algorithm: row-oriented (Banachiewicz, used in this implementation) vs column-oriented (Crout). Performance differs by cache access pattern.
- **Incomplete Cholesky.** Drops small entries during factorization to produce a sparse approximate factor. Used as a preconditioner for iterative solvers (e.g., conjugate gradient).
- **Rank-1 Update/Downdate.** Given the existing factor $L$ of $A$, efficiently compute the factor of $A + vv^T$ (or $A - vv^T$) in $O(n^2)$ without recomputing from scratch. Central to the square-root UKF.
- **Block Cholesky.** Exploits block structure in large matrices for better cache utilization and parallelism.

## Applications

- **Sigma point generation** — The Unscented Kalman Filter uses Cholesky decomposition of the covariance matrix $P = LL^T$ to spread sigma points along the principal uncertainty directions.
- **Solving symmetric positive-definite systems** — Given $Ax = b$, compute $L$, then solve $Ly = b$ (forward substitution) and $L^Tx = y$ (back substitution). Twice as fast as general LU.
- **Multivariate normal sampling** — To generate $x \sim \mathcal{N}(\mu, \Sigma)$, compute $L$ from $\Sigma$ and set $x = \mu + Lz$ where $z \sim \mathcal{N}(0, I)$.
- **Least squares** — The normal equations $A^TAx = A^Tb$ produce a symmetric positive-definite system solvable via Cholesky.
- **Optimization** — Newton-type methods with positive-definite Hessians use Cholesky to compute search directions.

## Connections to Other Algorithms

| Algorithm                                                       | Relationship                                                             |
|-----------------------------------------------------------------|--------------------------------------------------------------------------|
| [Unscented Kalman Filter](../filters/active/UnscentedKalmanFilter.md) | Uses Cholesky to generate sigma points from the covariance matrix |
| [Gaussian Elimination](GaussianElimination.md)                  | General-purpose alternative; does not exploit symmetry                    |

## References & Further Reading

- Golub, G.H. and Van Loan, C.F., *Matrix Computations*, 4th ed., Johns Hopkins University Press, 2013.
- Press, W.H. et al., *Numerical Recipes*, 3rd ed., Cambridge University Press, 2007 — Section 2.9.
