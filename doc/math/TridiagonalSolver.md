# Tridiagonal Solver (Thomas Algorithm)

## Overview & Motivation

A tridiagonal system has nonzeros only on the main diagonal and its two neighbours. Spline interpolation, implicit heat/diffusion steps and several smoothing problems all reduce to such systems. A dense solver wastes cubic time and quadratic memory on zeros; the **Thomas algorithm** is Gaussian elimination that skips every zero, so it runs in linear time using three short bands.

The elimination sweep depends only on the matrix, so it is performed once and reused: each new right-hand side costs two short sweeps and no division. This fits the common case where the matrix depends on fixed geometry (for example knot spacing) while the right-hand side changes per channel or per tick.

## Mathematical Theory

Let $A \in \mathbb{R}^{n \times n}$ be tridiagonal with sub-diagonal $a_k = A_{k,k-1}$ ($a_0$ unused), diagonal $b_k$ and super-diagonal $c_k = A_{k,k+1}$ ($c_{n-1}$ unused). The system $Ax = d$ reads

$$a_k x_{k-1} + b_k x_k + c_k x_{k+1} = d_k, \quad k = 0, \dots, n-1$$

### Forward elimination (matrix only)

Eliminating $a_k$ with the already-reduced row above leaves an upper bidiagonal system with unit diagonal. Define the pivots and modified super-diagonal

$$p_0 = b_0, \qquad p_k = b_k - a_k\, c'_{k-1}, \qquad c'_k = \frac{c_k}{p_k}$$

The reciprocals $1/p_k$ are stored, so $c'_k = c_k \cdot (1/p_k)$. This is the factorization $A = LU$ with $L$ lower bidiagonal (diagonal $p_k$, sub-diagonal $a_k$) and $U$ unit upper bidiagonal (super-diagonal $c'_k$).

### Forward substitution (right-hand side)

$$y_0 = \frac{d_0}{p_0}, \qquad y_k = \frac{d_k - a_k\, y_{k-1}}{p_k}$$

### Back substitution

$$x_{n-1} = y_{n-1}, \qquad x_k = y_k - c'_k\, x_{k+1}, \quad k = n-2, \dots, 0$$

Division by $p_k$ is replaced by multiplication with the stored reciprocal.

## Complexity Analysis

| Operation              | Time     | Space    | Notes                                        |
|------------------------|----------|----------|----------------------------------------------|
| Factorization          | $O(n)$   | $O(n)$   | $n$ divisions, one multiply-subtract per row |
| Solve (per RHS)        | $O(n)$   | $O(n)$   | About $3n$ multiply-adds, no division        |
| Dense LU, for contrast | $O(n^3)$ | $O(n^2)$ | Operates on all the zeros                    |

Storage is three bands of fixed capacity; nothing is allocated dynamically. Factoring once and solving $m$ right-hand sides costs $O(n + mn)$ with all divisions paid up front.

## Step-by-Step Walkthrough

Take $A = \operatorname{tridiag}(-1, 2, -1)$ with $n = 4$, so $a_k = c_k = -1$, $b_k = 2$.

| $k$ | $p_k$ | $1/p_k$ | $c'_k$ |
|-----|-------|---------|--------|
| 0   | $2$   | $1/2$   | $-1/2$ |
| 1   | $2 - (-1)(-1/2) = 3/2$ | $2/3$ | $-2/3$ |
| 2   | $2 - (-1)(-2/3) = 4/3$ | $3/4$ | $-3/4$ |
| 3   | $2 - (-1)(-3/4) = 5/4$ | $4/5$ | unused |

**Right-hand side** $d = (1, 0, 0, 1)$: forward $y = (1/2,\ 1/3,\ 1/4,\ 1)$; back substitution gives $x_3 = 1$, $x_2 = 1/4 + \tfrac34 = 1$, $x_1 = 1/3 + \tfrac23 = 1$, $x_0 = 1/2 + 1/2 = 1$, so $x = (1, 1, 1, 1)$.

**Right-hand side** $d = (1, 2, 3, 4)$ reuses the same table: forward $y = (1/2,\ 5/3,\ 7/2,\ 6)$; back substitution gives $x_3 = 6$, $x_2 = 7/2 + 9/2 = 8$, $x_1 = 5/3 + 16/3 = 7$, $x_0 = 1/2 + 7/2 = 4$, so $x = (4, 7, 8, 6)$.

**Verify:** row 1 of $Ax$ is $-4 + 14 - 8 = 2$ ✓ and row 3 is $-8 + 12 = 4$ ✓.

## Pitfalls & Edge Cases

- **No pivoting.** The algorithm is stable when $A$ is strictly diagonally dominant ($|b_k| > |a_k| + |c_k|$) or symmetric positive-definite; spline systems are both. For other tridiagonal matrices a zero or tiny pivot can appear even when $A$ is nonsingular, and growth can destroy accuracy. Use LU decomposition with partial pivoting instead.
- **Vanishing pivot.** If $|p_k| \le \varepsilon \cdot s$, where $\varepsilon$ is the machine epsilon and $s$ is the largest absolute entry among the $n$ rows, the factorization reports failure instead of dividing. The relative threshold keeps the test meaningful regardless of the matrix's scale.
- **Size limits.** A size of zero or one exceeding the compile-time capacity is rejected. Entries of the solution beyond $n$ are zero.
- **Failed factorization.** A solve is only meaningful after a successful factorization; refactor whenever the matrix changes.
- **Floating-point only.** Reciprocals and the epsilon test have no meaning for integer types.

## Variants & Generalizations

- **Cyclic tridiagonal.** Periodic boundary conditions add corner entries; the Sherman–Morrison formula reduces the solve to two tridiagonal solves.
- **Block tridiagonal.** Scalar pivots become small dense blocks, as in implicit multi-variable PDE steps.
- **Symmetric case.** When $a_k = c_{k-1}$, one band can be dropped and the factorization becomes $LDL^T$.
- **Parallel variants.** Cyclic reduction trades extra work for parallelism; irrelevant on a single embedded core.

## Applications

- **Cubic spline interpolation** — the second-derivative system at the knots is tridiagonal and diagonally dominant.
- **Implicit finite differences** — backward-Euler and Crank–Nicolson steps for 1-D heat and diffusion equations.
- **Smoothing** — penalized least-squares smoothers produce banded normal equations.

## Connections to Other Algorithms

| Algorithm                                         | Relationship                                                                 |
|---------------------------------------------------|------------------------------------------------------------------------------|
| [LU Decomposition](../solvers/LuDecomposition.md) | General fallback with partial pivoting; $O(n^3)$ and dense                   |
| [Triangular Solve](TriangularSolve.md)            | Same forward/back substitution idea, applied to full triangular factors           |
| [Levinson–Durbin](../solvers/LevinsonDurbin.md)   | Another structured solver, for Toeplitz systems                              |
| Cubic spline interpolation                        | Main consumer; plans the second derivatives by solving one tridiagonal system |

## References & Further Reading

- Thomas, L. H., "Elliptic Problems in Linear Difference Equations over a Network", Watson Scientific Computing Laboratory, Columbia University, 1949.
- Golub, G. H. & Van Loan, C. F., *Matrix Computations*, 4th ed., Johns Hopkins University Press, 2013, §4.3.
