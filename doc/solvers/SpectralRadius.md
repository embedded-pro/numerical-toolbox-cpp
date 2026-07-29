# Spectral Radius & Discrete Stability Margin

## Overview & Motivation

The spectral radius of a square matrix is the largest magnitude among all its eigenvalues. In discrete-time dynamical systems, the spectral radius of the state-transition matrix is the single most important scalar quantity: if it is strictly less than one, every state trajectory converges to zero; if it equals or exceeds one, at least one mode grows without bound or fails to decay. Rather than reporting only a binary stable/unstable flag, exposing the spectral radius and the complementary stability margin (one minus the spectral radius) gives the designer a quantitative handle on how much gain or perturbation the system can tolerate before losing stability.

## Mathematical Theory

### Characteristic Polynomial via Faddeev–LeVerrier

Given an $n \times n$ matrix $A$, the characteristic polynomial is

$$p(\lambda) = \det(\lambda I - A) = \lambda^n + c_{n-1}\lambda^{n-1} + \cdots + c_1\lambda + c_0.$$

The Faddeev–LeVerrier recurrence computes the coefficients $c_k$ without determinant expansion. Define auxiliary matrices $M_k$ by

$$M_1 = A, \quad c_{n-1} = -\operatorname{tr}(M_1),$$
$$M_k = A\bigl(M_{k-1} + c_{n-k+1}I\bigr), \quad c_{n-k} = -\frac{1}{k}\operatorname{tr}(M_k), \quad k = 2, \ldots, n.$$

This recurrence requires only matrix-matrix multiplication, scalar multiplication of a matrix by the identity, and the trace, making it suitable for static (stack-allocated) computation without a heap.

### Spectral Radius

Given the full set of eigenvalues $\{\lambda_i\}_{i=1}^{n}$ (roots of the characteristic polynomial),

$$\rho(A) = \max_{i} |\lambda_i|.$$

The roots are found numerically with the Durand–Kerner simultaneous root-finding iteration, which converges for arbitrary polynomials and requires only a fixed-size array of complex iterates.

### Schur Stability

A matrix $A$ is Schur-stable (the discrete-time analogue of Hurwitz stability) if and only if $\rho(A) < 1$. Every eigenvalue then lies strictly inside the complex unit disk, guaranteeing exponential decay of all modes.

### Stability Margin

$$\delta(A) = 1 - \rho(A).$$

A positive margin indicates Schur stability; $\delta = 0$ corresponds to marginal stability (an eigenvalue on the unit circle); $\delta < 0$ signals instability. The margin quantifies how far $\rho(A)$ is from the stability boundary.

## Complexity Analysis

| Case    | Time                        | Space        | Notes                                                                 |
|---------|-----------------------------|--------------|-----------------------------------------------------------------------|
| Best    | $O(n^3)$                    | $O(n^2)$     | Faddeev–LeVerrier dominates; two $n\times n$ temporaries on the stack |
| Average | $O(n^3 + n \cdot I)$        | $O(n^2 + n)$ | $I$ Durand–Kerner iterations, typically $\ll 200$                     |
| Worst   | $O(n^3 + n \cdot I_{\max})$ | $O(n^2 + n)$ | Near-repeated roots slow DK convergence                               |

The Faddeev–LeVerrier step executes $n$ matrix multiplications each costing $O(n^3)$, giving $O(n^4)$ total; for the small dimensions typical of embedded control matrices ($n \le 6$) this is negligible.

## Step-by-Step Walkthrough

Consider $A = \begin{pmatrix} 0 & -1 \\ 1 & 0 \end{pmatrix}$ (90° rotation, $\rho = 1$).

1. **k = 1**: $M_1 = A \cdot 0 + I \cdot c_0$. With $c_0 = 1$: $M_1 = I$. Then $c_1 = -\tfrac{1}{1}\operatorname{tr}(A \cdot M_1) = -\operatorname{tr}(A) = 0$.
2. **k = 2**: $M_2 = A(M_1 + c_1 I) = A \cdot I = A$. Then $c_2 = -\tfrac{1}{2}\operatorname{tr}(A \cdot A) = -\tfrac{1}{2}\operatorname{tr}(-I) = 1$.

Characteristic polynomial: $\lambda^2 + 0\lambda + 1 = \lambda^2 + 1$. Roots: $\pm i$, magnitudes both 1. $\rho = 1$.

## Pitfalls & Edge Cases

- **Near-repeated roots**: Durand–Kerner converges slowly when two or more eigenvalues are nearly identical. The stability conclusion (inside/outside unit disk) is unaffected, but the raw $\rho$ value may have larger numerical error.
- **Identity approximation in Matrix::Identity()**: This implementation builds its own exact identity to avoid reliance on library internals.
- **Ill-conditioned characteristic polynomials**: For large $n$ the polynomial coefficients can have widely different magnitudes, degrading root-finder precision. The algorithm is intended for small matrices ($n \le 6$) typical of discrete-time state-space models.
- **Size = 1 case**: Handled directly as $\rho = |a_{11}|$, bypassing polynomial computation.
- **Zero matrix**: $\rho = 0$, $\delta = 1$ (maximally stable). Handled correctly by the recurrence.

## Variants & Generalizations

- **Power iteration** computes only the dominant eigenvalue magnitude without the full spectrum; cheaper for large $n$ but provides no stability margin without additional eigenpair deflation.
- **QR algorithm** (Jacobi, Hessenberg QR) finds all eigenvalues more robustly for large or symmetric matrices; too expensive and heap-dependent for small embedded targets.
- **Continuous-time analogue**: Replace the unit-disk criterion with the left half-plane criterion; Routh–Hurwitz or Lyapunov-based approaches apply.

## Applications

- Verifying discrete-time state-feedback or observer designs remain within the unit disk after coefficient quantisation.
- Checking closed-loop stability after LQR or pole-placement synthesis before deployment.
- Computing gain/phase margins indirectly by perturbing the loop gain and re-evaluating $\rho$.
- Monitoring on-line whether a time-varying system's frozen eigenvalues have drifted outside the unit disk.

## Connections to Other Algorithms

- Uses **Durand–Kerner** (polynomial root-finder) for the eigenvalue computation stage.
- The characteristic polynomial recurrence is mathematically related to **Leverrier's method** for matrix inverses and adjoints.
- The Schur-stability criterion is the discrete-time counterpart of the Hurwitz criterion used in continuous-time control analysis.
- The stability margin scalar is a prerequisite for **TESTING.md** evaluation metrics that require a quantitative stability distance.

## References & Further Reading

- Faddeev, D. K. & LeVerrier, U. J. F. — classical matrix identity; see G. H. Golub & C. F. Van Loan, *Matrix Computations*, 4th ed., Section 7.3.
- Durand, E. (1960) and Kerner, I. O. (1966) — simultaneous root-finding; see *Numerical Recipes in C*, 3rd ed., Section 9.5.
- Schur stability: K. J. Åström & B. Wittenmark, *Computer-Controlled Systems*, 3rd ed., Chapter 3.
