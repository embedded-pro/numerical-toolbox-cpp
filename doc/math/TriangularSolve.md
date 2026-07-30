# Triangular Solve

## Overview & Motivation

Nearly every dense linear solver ends with a triangular system. Gaussian elimination reduces `Ax = b` to an upper-triangular `Ux = c`; QR factorization solves least squares through `Rx = Qᵀb`; Cholesky solves two triangular systems back-to-back. `SolveUpperTriangular` is the shared back-substitution kernel these routines call so the algorithm lives in exactly one place.

## Mathematical Theory

Given an upper-triangular matrix $R \in \mathbb{R}^{n \times n}$ (entries below the diagonal ignored) and a right-hand side $c$, back-substitution solves $Rx = c$ bottom-up:

$$x_i = \frac{1}{r_{ii}}\left(c_i - \sum_{j=i+1}^{n} r_{ij}\, x_j\right), \quad i = n, n-1, \dots, 1$$

Each unknown depends only on those already computed, so a single reverse sweep suffices.

## Complexity Analysis

| Operation             | Time     | Space  | Notes                          |
|-----------------------|----------|--------|--------------------------------|
| SolveUpperTriangular  | $O(n^2)$ | $O(n)$ | One reverse sweep, stack output |

## Step-by-Step Walkthrough

For $R = \begin{bmatrix}2 & -1 & 3 \\ 0 & 4 & 1 \\ 0 & 0 & 5\end{bmatrix}$, $c = [10,\, -5,\, 15]^\top$:

1. $x_3 = 15 / 5 = 3$
2. $x_2 = (-5 - 1\cdot 3)/4 = -2$
3. $x_1 = (10 - (-1)(-2) - 3\cdot 3)/2 = 1$ ⇒ $x = [1,\, -2,\, 3]^\top$.

## Pitfalls & Edge Cases

**Singular / near-zero pivot** — a zero diagonal entry makes the system unsolvable. The routine asserts `|r_{ii}| > 0` via `really_assert`; callers (`GaussianElimination`, `QrDecomposition`) detect rank deficiency before reaching it.

**Generic on `T`** — the kernel is templated on any supported numeric type (`float`, `Q15`, `Q31`) because Gaussian elimination is instantiated for all three; it uses `math::ToFloat` only for the pivot assertion.

## Variants & Generalizations

A lower-triangular forward-substitution is the mirror image (top-down sweep) and can be added when Cholesky/LU forward solves need it. Block triangular solves generalise this to matrix right-hand sides.

## Applications

- **Gaussian elimination** — final back-substitution after forward elimination.
- **QR least squares** — solving `Rx = Qᵀb`.
- Any factor-then-solve routine producing a triangular factor.

## Connections to Other Algorithms

Shared by `solvers::GaussianElimination` and `solvers::QrDecomposition`. Operates on `math::Matrix` / `math::Vector`.

## References & Further Reading

- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., §3.1 (triangular systems)
- Trefethen, L. N. & Bau, D., "Numerical Linear Algebra", Lecture 17
