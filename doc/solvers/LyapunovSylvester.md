# Lyapunov and Sylvester Equation Solvers

## Overview & Motivation

The Sylvester equation $AX + XB = C$ and its symmetric specialisations — the continuous Lyapunov
$AX + XA^\top = -Q$ and the discrete Lyapunov $AXA^\top - X = -Q$ — are the central matrix
equations of stability analysis, gramian computation, and robust-control synthesis. A
positive-definite Lyapunov solution is a formal certificate that the system $\dot{x} = Ax$ is
asymptotically stable; the same equations produce the **controllability and observability gramians**
that measure how effectively a linear system can be driven and observed.

## Mathematical Theory

### Sylvester Equation

Given matrices $A \in \mathbb{R}^{N \times N}$, $B \in \mathbb{R}^{M \times M}$, and
$C \in \mathbb{R}^{N \times M}$, the Sylvester equation is

$$AX + XB = C,$$

with unknown $X \in \mathbb{R}^{N \times M}$.  The equation has a unique solution if and only if
$A$ and $-B$ share no eigenvalue, equivalently $\lambda_i(A) + \lambda_j(B) \neq 0$ for all
$i, j$.

### Kronecker Vectorisation

Stacking $X$ column-by-column into $\operatorname{vec}(X) \in \mathbb{R}^{NM}$ and applying
the identities $\operatorname{vec}(AXI) = (I_M \otimes A)\operatorname{vec}(X)$ and
$\operatorname{vec}(IXB) = (B^\top \otimes I_N)\operatorname{vec}(X)$ converts the matrix
equation into the standard linear system

$$\underbrace{(I_M \otimes A + B^\top \otimes I_N)}_{K \in \mathbb{R}^{NM \times NM}}
\operatorname{vec}(X) = \operatorname{vec}(C).$$

The $(iN+k, jN+l)$ entry of $K$ is $\delta_{ij} A_{kl} + B_{ji} \delta_{kl}$.  The system
is solved by Gaussian elimination with partial pivoting; a near-zero pivot signals the
unsolvable (degenerate eigenvalue) case.

### Continuous Lyapunov Equation

Setting $B = A^\top$ and $C = -Q$ reduces the Sylvester equation to the continuous Lyapunov form

$$AX + XA^\top = -Q.$$

When $A$ is Hurwitz (all eigenvalues in the open left half-plane) and $Q$ is symmetric
positive semi-definite, the unique solution $X$ is symmetric positive definite.

### Discrete Lyapunov Equation

The discrete counterpart

$$AXA^\top - X = -Q$$

is solved by the Kronecker system $(A \otimes A - I_{N^2})\operatorname{vec}(X) = -\operatorname{vec}(Q)$,
with the $(iN+k, jN+l)$ entry of $A \otimes A - I$ equal to $A_{ij}A_{kl} - \delta_{ij}\delta_{kl}$.
When $A$ is Schur-stable ($\rho(A) < 1$) and $Q \succeq 0$, the unique solution equals the
controllability gramian $X = \sum_{k=0}^{\infty} A^k Q (A^\top)^k$.

## Complexity Analysis

| Case                          | Time           | Space                       | Notes                                                         |
|-------------------------------|----------------|-----------------------------|---------------------------------------------------------------|
| $N{=}M{=}2$                   | $O(64)$ flops  | $O(NM)^2 = 16$ floats for K | 4×4 linear system                                             |
| $N{=}M{=}3$                   | $O(729)$ flops | 81 floats for K             | 9×9 linear system                                             |
| General $N{=}M$               | $O(N^6)$ flops | $O(N^4)$ words              | Gaussian elimination on the $N^2 \times N^2$ Kronecker system |
| Alternative (Bartels–Stewart) | $O(N^3)$       | $O(N^2)$                    | Requires Schur decomposition; preferred for $N \geq 6$        |

For the embedded sizes ($N \leq 5$) targeted by this library the Kronecker approach is negligibly
more expensive than Bartels–Stewart and avoids the full Schur machinery.

## Step-by-Step Walkthrough

Solve $AX + XB = C$ for the $2 \times 2$ case
$A = \bigl[\begin{smallmatrix}-3&1\\0&-2\end{smallmatrix}\bigr]$,
$B = \bigl[\begin{smallmatrix}1&2\\0&1\end{smallmatrix}\bigr]$,
$C = I_2$.

**Build $K$** (the $4 \times 4$ Kronecker matrix):

$$K = \begin{pmatrix}-2&1&0&0\\0&-1&0&0\\2&0&-2&1\\0&2&0&-1\end{pmatrix}.$$

**Build $\operatorname{vec}(C) = (1, 0, 0, 1)^\top$.**

**Gaussian elimination** with partial pivoting produces $X_{00} = -0.5$, $X_{10} = 0$,
$X_{01} = -1$, $X_{11} = -1$.

**Verify:** $AX + XB = I_2$. ✓

## Pitfalls & Edge Cases

- **Unsolvable case:** if $\lambda_i(A) + \lambda_j(B) = 0$ (continuous) or
  $\lambda_i(A)\lambda_j(B) = 1$ (discrete) for any pair, $K$ is singular and no unique solution
  exists.  The solver returns `false` and leaves the solution unchanged.
- **Ill-conditioning:** near-degenerate eigenvalue pairs yield large solution norms and reduced
  accuracy.  Monitor the residual $\|AX + XB - C\|$ after solving.
- **Symmetry:** the continuous Lyapunov solution is guaranteed symmetric only when $Q$ is
  symmetric; numerical symmetry should be enforced by the caller if required downstream.

## Variants & Generalizations

- **Bartels–Stewart algorithm:** reduces to quasi-upper-triangular (Schur) form via the double-shift
  QR algorithm, then back-substitutes column by column in $O(N^3)$.  Preferred for $N \geq 6$.
- **Smith iteration:** fixed-point iteration $X_{k+1} = AX_k A^\top + Q$ for the discrete
  Lyapunov equation; simple but slow for $\rho(A)$ close to 1.
- **Cholesky factor updates:** propagate a Cholesky factor of $X$ for guaranteed positive-definiteness
  in square-root filtering (see `SquareRootKalmanFilter`).

## See Also

`GaussianElimination`, `QrDecomposition`, `DiscreteAlgebraicRiccatiEquation`, `SpectralRadius`.
