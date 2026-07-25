# Matrix Exponential — Overview

## What it is
The matrix generalization of `e^x`. For a square matrix `A`, `expm(A) = I + A + A²/2! + A³/3! + …`.
It is the exact solution operator of the linear ODE `ẋ = Ax`: `x(t) = expm(A·t)·x(0)`.

## Why it matters (embedded)
It is the engine behind **exact discretization** — turning a continuous-time state-space model into
the discrete `(A_d, B_d)` an MCU actually runs (see `ContinuousToDiscrete`). It also appears in
continuous Gramians and on-device linear-system simulation. Getting it accurate and bounded is a
prerequisite for correct control deployment.

## How it works (intuition)
Naively summing the Taylor series is inaccurate: for large `‖A‖` the intermediate terms grow into a
huge "hump" before cancelling. The robust recipe is **scaling and squaring**: divide `A` by a power
of two until it is small, approximate `expm` of the shrunken matrix with a **Padé rational
approximant** (more accurate than a truncated series for the same cost), then repeatedly square the
result to undo the scaling, since `expm(A) = expm(A/2^s)^(2^s)`.

## Key parameters
- **Padé order** — degree of the rational approximant (accuracy vs matrix-multiply count).
- **Scaling exponent `s`** — chosen from `‖A‖` so the Padé argument stays small.

## Reference
C. Moler, C. Van Loan, "Nineteen Dubious Ways to Compute the Exponential of a Matrix, Twenty-Five
Years Later," *SIAM Review*, 45(1), 2003; N. J. Higham, "The scaling and squaring method for the
matrix exponential revisited," 2005.

## See also
`ContinuousToDiscrete` (item 30, the main consumer), `GaussianElimination` / `LuDecomposition`
(the Padé solve), `LyapunovSylvester` (item 31, also matrix-function based).
