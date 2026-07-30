# Givens Rotation

## Overview & Motivation

A Givens rotation zeros a single matrix entry by rotating two rows in a plane, leaving all others untouched. Because it touches only two rows, it is the tool of choice for *incremental* linear algebra: streaming a new row into an existing QR factor, sparse triangularization, and the implicit-shift sweeps of QR/SVD eigen-iterations. `ComputeGivens` derives the rotation coefficients; `ApplyGivens` applies them to a scalar pair.

## Mathematical Theory

Given two scalars $a$ (the value to keep) and $b$ (the value to zero), the rotation

$$G = \begin{bmatrix} c & s \\ -s & c \end{bmatrix}, \quad c = \frac{a}{r},\; s = \frac{b}{r},\; r = \sqrt{a^2 + b^2}$$

is orthogonal ($c^2 + s^2 = 1$) and satisfies

$$G \begin{bmatrix} a \\ b \end{bmatrix} = \begin{bmatrix} r \\ 0 \end{bmatrix}$$

Applied across a pair of rows, it zeros the target entry while preserving Euclidean length.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                   |
|----------------|--------|--------|-----------------------------------------|
| ComputeGivens  | $O(1)$ | $O(1)$ | One `sqrt`, two divides                 |
| ApplyGivens    | $O(1)$ | $O(1)$ | Rotates one scalar pair                 |
| Rotate two rows| $O(n)$ | $O(1)$ | `ApplyGivens` across `n` columns        |

## Step-by-Step Walkthrough

`ComputeGivens(3, 4)`: $r = 5$, $c = 0.6$, $s = 0.8$. Applying to $(x, y) = (3, 4)$: $x' = 0.6\cdot 3 + 0.8\cdot 4 = 5$, $y' = -0.8\cdot 3 + 0.6\cdot 4 = 0$ — the second component is annihilated and the norm is preserved.

## Pitfalls & Edge Cases

**Degenerate pair** — when $a = b = 0$ the rotation is undefined; `ComputeGivens` returns the identity $(c, s) = (1, 0)$ so applying it is a safe no-op.

**Float-only** — `static_assert(std::is_floating_point_v<T>)`.

## Variants & Generalizations

Householder reflectors (`math::HouseholderTransform`) zero an entire sub-column at once and are cheaper for dense factorization; Givens wins when only one entry (or one streamed row) changes. Fast/"square-root-free" Givens variants trade the `sqrt` for extra bookkeeping.

## Applications

- **Streaming QR update** — rotate a new row into an existing `R` (`QrDecomposition::GivensUpdateRow`).
- **QR / SVD iterations** — implicit-shift bulge chasing.
- **Sparse triangularization** — zero isolated entries without touching the rest.

## Connections to Other Algorithms

Used by `solvers::QrDecomposition`; complements `math::HouseholderTransform`.

## References & Further Reading

- Givens, W., "Computation of Plane Unitary Rotations Transforming a General Matrix to Triangular Form", SIAM J. Appl. Math. 6(1), 1958
- Golub, G. H. & Van Loan, C. F., "Matrix Computations", 4th ed., §5.1 (Givens rotations)
