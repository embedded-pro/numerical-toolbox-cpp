# Incremental QR Least Squares — Overview

## What it is
A least-squares solver that takes measurements one row at a time, keeps only a small square summary,
and at the end returns the best-fitting parameters — together with which parameter combinations the
data could actually determine.

## Why it matters (embedded)
Identification and calibration runs produce long logs: thousands of samples of motor current versus
acceleration, or joint torques versus motion. Storing them all does not fit in a microcontroller, and the
textbook shortcut (normal equations) squares the condition number, which `float` cannot afford. Rotating
each row into a triangular summary keeps memory fixed and accuracy close to a full QR. Real systems are
often rank-deficient — some parameters never show up in the data, or only in fixed combinations — so the
solver must report what is identifiable instead of returning garbage.

## How it works (intuition)
Keep an upper-triangular matrix and a right-hand-side vector. For each new row, a sequence of plane
rotations zeroes its entries one by one against the triangle; what remains of the right-hand side is
pure misfit and is just accumulated as a squared error. Solving at the end means decomposing the small
triangle into singular directions, inverting the strong ones and dropping the weak ones: the result is
the minimum-norm solution, and the kept directions are the identifiable combinations.

## Key parameters
- **Parameter count `P`** — compile-time; memory is `P²`.
- **Relative rank tolerance** — which singular values count as identifiable.

## Reference
G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., §5.1.8 (Givens QR) and §5.5 (rank-deficient
least squares); Å. Björck, *Numerical Methods for Least Squares Problems* (1996), Ch. 2.

## See also
`RecursiveLeastSquares` (online, forgetting), `LinearRegression` (batch with stored data),
`TotalLeastSquares`, `SingularValueDecomposition`, `GivensRotation`; robotics-toolbox-cpp's dynamic
parameter identification, which feeds the inertial regressor into this solver.
