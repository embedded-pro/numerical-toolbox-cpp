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

## Pitfalls & Edge Cases

- **Non-positive-definite input.** If $A$ is not positive-definite, a diagonal element under the square root will be negative, producing NaN. Always verify the input or add a small regularization $A + \epsilon I$.
- **Numerical precision.** In fixed-point arithmetic, the intermediate sums and square root can lose precision. Use float for the decomposition and convert back if needed.
- **Zero diagonal.** A zero or near-zero diagonal in $L$ indicates a singular or nearly singular matrix.

## Connections to Other Algorithms

| Algorithm                                                       | Relationship                                                             |
|-----------------------------------------------------------------|--------------------------------------------------------------------------|
| [Unscented Kalman Filter](../filters/active/UnscentedKalmanFilter.md) | Uses Cholesky to generate sigma points from the covariance matrix |
| [Gaussian Elimination](GaussianElimination.md)                  | General-purpose alternative; does not exploit symmetry                    |

## References & Further Reading

- Golub, G.H. and Van Loan, C.F., *Matrix Computations*, 4th ed., Johns Hopkins University Press, 2013.
- Press, W.H. et al., *Numerical Recipes*, 3rd ed., Cambridge University Press, 2007 — Section 2.9.
