# Residual-Form Kalman Update — Overview

## What it is
The measurement-update half of a Kalman filter, packaged as a standalone step. The caller states how far
the measurement is from the prediction (the residual), how that measurement depends on the state and how
noisy it is. The step corrects the state and its uncertainty, or refuses if the measurement is
implausible.

## Why it matters (embedded)
Real estimators rarely fit a fixed "measure, subtract, correct" mould. A bearing of 179° against a
prediction of −179° is a 2° error, not 358°. A robot seeing one landmark among many needs a different
measurement model each time. A reflection or a mismatched landmark must be thrown out before it
corrupts the estimate. Letting the caller form the residual and checking it against the predicted
uncertainty covers all three in a few dozen operations for small states.

## How it works (intuition)
Combining the predicted uncertainty with the measurement noise gives the expected spread of the residual.
Dividing the actual residual by that spread (in the matrix sense) yields the normalised innovation
squared — how many "standard deviations" away the measurement is. If that exceeds a χ² gate the
measurement is ignored. Otherwise the Kalman gain blends prediction and measurement, and the covariance is
updated in Joseph form, which keeps it a valid covariance even with rounding.

## Key parameters
- **residual** `z ⊖ h(x)` — formed by the caller (wrapped for angles).
- **measurement Jacobian `H`** and **noise `R`**.
- **gate** — χ² threshold on the NIS (e.g. 9.21 for 2-D at 99 %).

## Reference
P. S. Maybeck, *Stochastic Models, Estimation, and Control*, Vol. 1 (1979), Ch. 5 (Joseph form);
Y. Bar-Shalom, X.-R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation* (2001),
Ch. 5 (validation gates).

## See also
`ExtendedKalmanFilter` (fixed-model filter), `ConsistencyMetrics` (NIS/NEES), `AngleUtilities` (item 54);
robotics-toolbox-cpp's odometry fusion (E1) and landmark localization (E3).
