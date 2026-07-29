# Condition Number

## Overview & Motivation

A matrix can be mathematically invertible yet catastrophically sensitive to perturbations in its entries or in the right-hand side of a linear system. This sensitivity, called ill-conditioning, is the root cause of accumulated floating-point error in solvers, regression, and Kalman-filter covariance updates. Measuring it before committing to a solve reveals whether the result can be trusted or whether regularisation, pivoting, or reformulation is warranted.

The condition number is the canonical scalar summary of ill-conditioning. Because it requires the norm of the inverse, it depends on a linear solver — which is why it lives in the `solvers` layer, on top of the `math` norms, rather than in `math` itself.

## Mathematical Theory

For an invertible square matrix $A \in \mathbb{R}^{N \times N}$ and a chosen matrix norm $\|\cdot\|$:

$$\kappa(A) = \|A\| \cdot \|A^{-1}\|$$

The condition number bounds the relative error amplification in the solution $\mathbf{x}$ of $A\mathbf{x} = \mathbf{b}$ due to a perturbation $\delta\mathbf{b}$:

$$\frac{\|\delta\mathbf{x}\|}{\|\mathbf{x}\|} \le \kappa(A) \cdot \frac{\|\delta\mathbf{b}\|}{\|\mathbf{b}\|}$$

$\kappa(A) \ge 1$ always; $\kappa(A) = 1$ only for scalar multiples of orthogonal matrices. A singular matrix has $\kappa(A) = \infty$. The estimate here uses the 1-norm.

### Computing the Inverse via Column-wise Solve

The norm of $A^{-1}$ is obtained without forming an explicit inverse as a first-class object: solving $AX = I$ column by column with partial-pivoting Gaussian elimination yields the inverse columns, from which the 1-norm is accumulated. This mirrors the embedded convention of reusing the existing solver rather than adding a dedicated inversion routine.

## Complexity Analysis

| Operation       | Time     | Space    | Notes                                         |
|-----------------|----------|----------|-----------------------------------------------|
| ConditionNumber | $O(N^3)$ | $O(N^2)$ | Dominated by the $N$-column solve of $AX = I$ |

## Step-by-Step Walkthrough

Matrix $A = \begin{bmatrix}3 & 1 \\ 1 & 2\end{bmatrix}$, 1-norm condition number:

1. $\|A\|_1 = \max(4, 3) = 4$
2. Solve $A\mathbf{x} = \mathbf{e}_0$: $\mathbf{x}_0 = [0.4,\, -0.2]^\top$
3. Solve $A\mathbf{x} = \mathbf{e}_1$: $\mathbf{x}_1 = [-0.2,\, 0.6]^\top$
4. $A^{-1} = \begin{bmatrix}0.4 & -0.2 \\ -0.2 & 0.6\end{bmatrix}$, so $\|A^{-1}\|_1 = \max(0.6, 0.8) = 0.8$
5. $\kappa_1(A) = 4 \times 0.8 = 3.2$

## Pitfalls & Edge Cases

**Singular matrices** — a row of all zeros makes the system unsolvable. The singularity check fires before entering Gaussian elimination, returning an empty result rather than dividing by zero.

**Near-singular matrices** — the condition number can be astronomically large without any row being identically zero. Partial-pivoting elimination still completes, and the returned value reflects the ill-conditioning faithfully.

**Norm choice** — the 1-norm condition number is a practical upper bound on the spectral condition number $\kappa_2$; it is cheaper by orders of magnitude because it avoids the SVD.

## Variants & Generalizations

The **spectral condition number** ($\kappa_2$, ratio of largest to smallest singular value) is the tightest but requires an SVD. The **reciprocal condition number** ($\text{RCOND} = 1/\kappa$) avoids overflow when $\kappa$ is very large; LAPACK-style solvers return this form and compare against machine epsilon. For **positive-definite** systems, Cholesky factorisation supplies the same column-wise solves at half the cost.

## Applications

- **Solver validation**: if $\kappa(A)\cdot\epsilon_{\text{mach}} \gtrsim 1$, the solution has no reliable digits.
- **Kalman filter covariance**: monitoring $\kappa(P)$ detects numerical collapse of the covariance.
- **Regression**: the design-matrix condition number governs least-squares sensitivity to noise.
- **Control design**: ill-conditioned system matrices signal near-uncontrollability or near-unobservability.

## Connections to Other Algorithms

Gaussian elimination with partial pivoting (`solvers::GaussianElimination`) supplies the column-wise solves of $AX = I$. The matrix 1-norm (`math::OneNorm`) supplies both factors of the product. Cholesky decomposition (`solvers::CholeskyDecomposition`) is the positive-definite alternative for the inverse solve.

## References & Further Reading

- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., Chapter 3 (Gaussian elimination)
- Higham, N. J., "Accuracy and Stability of Numerical Algorithms", 2nd ed., Chapter 6 (condition numbers)
- Trefethen, L. N. & Bau, D., "Numerical Linear Algebra", Lecture 12 (conditioning)
