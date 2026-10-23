# Tridiagonal Solver (Thomas Algorithm) — Overview

## What it is
A linear solver specialised for matrices whose only nonzeros are on the main diagonal and its two
neighbours. It is Gaussian elimination that skips every zero, so it runs in linear time.

## Why it matters (embedded)
Spline interpolation through many points, implicit heat/diffusion steps and several smoothing
problems produce tridiagonal systems. A dense solver wastes cubic time and quadratic memory on zeros; the
Thomas algorithm needs three short arrays and a few multiply-adds per row, so a 50-knot spline is
re-planned in microseconds.

## How it works (intuition)
Walk down the rows once, eliminating the sub-diagonal entry of each row with the row above; the system
becomes upper bidiagonal. Walk back up, solving one unknown per row. The downward sweep depends only on
the matrix, so it is done once; each new right-hand side then costs only the two short sweeps.

## Key parameters
- **Size `n`** — up to the compile-time capacity.
- **Diagonal dominance** — the condition under which skipping pivoting is safe.

## Reference
L. H. Thomas, "Elliptic problems in linear difference equations over a network," Watson Sci. Comput.
Lab. Report, Columbia University, 1949; G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed.,
§4.3.

## See also
`TriangularSolve` (the triangular sibling), `LuDecomposition` (general systems, pivoting), `CubicSpline`
(#61, the main consumer), `Toeplitz` and `LevinsonDurbin` (other structured solvers).
