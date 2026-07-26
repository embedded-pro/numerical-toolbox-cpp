# Polynomial Least-Squares Fitting — Overview

## What it is
A batch estimator that fits a degree-`d` polynomial `p(x) = c₀ + c₁x + … + c_d xᵈ` to a set of
`(x, y)` samples by minimizing the sum of squared residuals. It is the classic *linear*
least-squares problem: linear in the coefficients even though the model curve is not a line.

## Why it matters (embedded)
Sensor-calibration curves, thermistor/ADC linearization, drift and trend modeling, and compact
lookup-table replacement all reduce to fitting a low-order polynomial. It runs offline (at
commissioning or on a captured buffer) and produces a handful of coefficients that evaluate in a
few multiply-adds at runtime.

## How it works (intuition)
Stack the samples into a **Vandermonde** matrix `V` whose columns are successive powers of `x`.
The best-fit coefficients solve the **normal equations** `(VᵀV) c = Vᵀy`. Because `VᵀV` is small
(`(d+1)×(d+1)`) and symmetric positive-definite, a Cholesky or Gaussian solve returns the
coefficients directly — no iteration.

## Key parameters
- **Degree `d`** — model order; higher fits more wiggles but conditions worse and can overfit.
- **Sample count** — must be `≥ d+1`; more samples average out measurement noise.
- **Centering/scaling** — normalizing `x` is essential for numerical health at higher degree.

## Reference
W. H. Press, S. A. Teukolsky, W. T. Vetterling, B. P. Flannery, *Numerical Recipes*, Ch. 15
(Modeling of Data — general linear least squares).

## See also
`LinearRegression` (multivariate linear fit), `solvers::CholeskyDecomposition`,
`SavitzkyGolayFilter` (local polynomial smoothing), `TotalLeastSquares` (errors-in-variables).
