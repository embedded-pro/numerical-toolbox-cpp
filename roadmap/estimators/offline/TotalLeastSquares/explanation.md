# Total Least Squares — Overview

## What it is
An errors-in-variables fitting method: it finds the model that best explains the data when **both**
the inputs (regressors) and the outputs are corrupted by noise. Where ordinary least squares
assumes the inputs are exact and only `y` is noisy, TLS treats every measurement as uncertain.

## Why it matters (embedded)
Real sensors are noisy on every channel. Calibrating one measured quantity against another
(e.g. two drifting sensors, current-vs-torque, strain-vs-force) is an errors-in-variables problem;
using OLS there produces a systematically **biased** slope. TLS removes that bias, giving more
accurate calibration constants and system-identification parameters.

## How it works (intuition)
Geometrically, OLS minimizes vertical distances to the fitted line; TLS minimizes the
**perpendicular** distances. Algebraically, stack the inputs and output into one augmented matrix
`[A | b]` and take its singular value decomposition. The direction of **least variance** — the
right-singular vector of the smallest singular value — defines the hyperplane the data hugs, and
the coefficients drop out of that vector.

## Key parameters
- **Column scaling** — inputs and output must be normalized to comparable magnitudes first.
- **Singular-value gap** — the separation of the two smallest σ indicates how well-posed the fit is.

## Reference
G. H. Golub, C. F. Van Loan, "An Analysis of the Total Least Squares Problem,"
*SIAM J. Numer. Anal.*, 17(6), 1980.

## See also
`SingularValueDecomposition` (the numerical engine), `PolynomialFitting` / `LinearRegression`
(ordinary least-squares counterparts), `estimators/offline`.
