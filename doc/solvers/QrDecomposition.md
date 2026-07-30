# QR Decomposition

## Overview & Motivation

The QR decomposition factors a matrix $A$ into an orthogonal factor $Q$ and an upper-triangular
factor $R$. It is the numerically robust foundation for least-squares regression, eigenvalue
computation, and square-root filtering. In embedded systems, fitting an overdetermined model
(more sensor measurements than unknowns) arises constantly in calibration, system identification,
and sensor fusion. Solving the least-squares problem via QR avoids forming the normal equations
$A^\top A x = A^\top b$, whose condition number is the square of $A$'s — a critical advantage
when working with reduced word lengths.

## Mathematical Theory

### Definitions

For an $m \times n$ matrix $A$ with $m \geq n$, the thin QR factorization is

$$A = Q R,$$

where $Q \in \mathbb{R}^{m \times n}$ has orthonormal columns ($Q^\top Q = I_n$) and
$R \in \mathbb{R}^{n \times n}$ is upper triangular.

### Householder Reflectors

A Householder reflector is a symmetric, orthogonal matrix of the form

$$H = I - \beta v v^\top, \quad \beta = \frac{2}{\|v\|^2},$$

chosen so that $H x = \sigma e_1$ for a target vector $x$, where $\sigma = -\operatorname{sign}(x_1)\|x\|$.
The sign convention $\sigma = -\operatorname{sign}(x_1)\|x\|$ avoids catastrophic cancellation
when computing $v_1 = x_1 - \sigma$.

Applying $k$ successive reflectors $H_k, \ldots, H_1$ to $A$ triangularizes it:

$$H_n \cdots H_1 A = R \implies A = H_1 \cdots H_n R = QR.$$

The reflectors are stored implicitly: below the diagonal of the working matrix, alongside a
scalar $\beta_k$ per column, consuming only $O(mn)$ words.

### Givens Rotations for Streaming Updates

A Givens rotation

$$G(i,j,\theta) = \begin{pmatrix} c & s \\ -s & c \end{pmatrix}, \quad c = \cos\theta,\ s = \sin\theta$$

zeros a single sub-diagonal entry by rotating two rows. When one new row streams in after an
existing factorization, $n$ plane rotations update $R$ in $O(n^2)$ flops — far cheaper than
re-factorizing the augmented matrix from scratch.

### Least-Squares Solve

Given the factorization $A = QR$, the least-squares minimum-norm solution is

$$x = R^{-1} Q^\top b,$$

computed as: (1) multiply $b \leftarrow Q^\top b$ by applying the stored reflectors in order
without forming $Q$ explicitly, then (2) back-substitute $R x = (Q^\top b)_{1:n}$.

## Complexity Analysis

| Case    | Time                          | Space    | Notes                                           |
|---------|-------------------------------|----------|-------------------------------------------------|
| Best    | $O(mn^2)$                     | $O(mn)$  | Householder on a well-conditioned dense $A$     |
| Average | $O(2mn^2 - \tfrac{2}{3}n^3)$  | $O(mn)$  | Standard flop count from Golub & Van Loan       |
| Worst   | same as average (no pivoting) | $O(mn)$  | Rank-deficient case detected but not re-ordered |
| Update  | $O(n^2)$                      | $O(n^2)$ | Single Givens row update; no re-factor          |

All memory is stack-allocated via `std::array`; the factor is stored in-place over the input.

## Step-by-Step Walkthrough

Factor $A = \begin{pmatrix} 12 & -51 \\ 6 & 167 \\ -4 & 24 \end{pmatrix}$ (first two columns of the
Golub & Van Loan example).

**Column 0 ($k=0$):** Extract $x = (12, 6, -4)^\top$, $\|x\| = 14$. Choose $\sigma = -14$
(sign rule), $v_0 = 12 + 14 = 26$, $v = (26, 6, -4)^\top / 26$, $\beta = 2/\|v\|^2$.
After applying $H_0$: $R_{00} = 14$, zeros below.

**Column 1 ($k=1$):** Repeat on the $2 \times 1$ trailing sub-column.

The upper-triangular $R$ then has diagonal magnitudes $\{14, 175\}$ for this sub-matrix.

## Pitfalls & Edge Cases

- **Sign convention**: always use $\sigma = -\operatorname{sign}(x_1)\|x\|$ to prevent the
  subtraction $x_1 - \sigma$ from cancelling when $x_1 > 0$.
- **Rank deficiency**: a near-zero diagonal entry in $R$ signals a dependent column; the
  factorization stops and returns false rather than dividing by zero.
- **Accumulating $Q$**: forming $Q$ explicitly costs an extra $O(mn^2)$ pass and
  $O(mn)$ storage; `ApplyQtranspose` avoids this for solves.
- **Streaming updates**: Givens rotations maintain numerical orthogonality one rotation at a time;
  round-off does not accumulate as severely as re-using a stale Householder factor.
- **Square matrices**: thin QR with $m = n$ still works; $Q$ is $n \times n$ orthogonal.

## Variants & Generalizations

- **Column pivoting** (QRP): permutes columns to move the largest-norm column to the front,
  yielding a rank-revealing factorization useful for near-rank-deficient $A$.
- **Gram-Schmidt (MGS)**: equivalent in exact arithmetic but numerically inferior; only suitable
  when $A$ is already well-conditioned.
- **Block Householder**: batches several reflectors into a $WY$ representation for cache-efficient
  BLAS-3 level operations on large matrices.

## Applications

- Overdetermined least-squares regression: calibration curves, polynomial fits, sensor fusion.
- Eigenvalue computation: the QR algorithm iterates $A_{k+1} = R_k Q_k$ to convergence.
- Square-root Kalman filtering: the covariance propagation uses a QR update of the augmented
  square-root factor rather than an explicit $PFP^\top + Q$ product.
- Singular Value Decomposition: bidiagonalizes $A$ via two-sided Householder as a first step.

## Connections to Other Algorithms

- Provides the factorization underpinning `SingularValueDecomposition` (Golub-Kahan) and the
  symmetric eigenvalue solver (Jacobi / tridiagonal QR).
- The back-substitution step is shared with `GaussianElimination`'s upper-triangular solve.
- Supersedes the normal equations used by `LinearRegression` for ill-conditioned problems.
- The square-root Kalman filter stores $P^{1/2}$ and updates it with a Givens-based QR update.

## References & Further Reading

- A. S. Householder, "Unitary Triangularization of a Nonsymmetric Matrix," *JACM*, 5(4), 1958.
- G. H. Golub & C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins UP, 2013, Ch. 5.
- W. H. Press et al., *Numerical Recipes in C*, 3rd ed., Cambridge UP, 2007, §2.10.
