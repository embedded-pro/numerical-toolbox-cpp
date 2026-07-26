# Luenberger Observer — Overview

## What it is
A deterministic state estimator. Given a plant model `(A, B, C)` and its inputs `u` and outputs
`y`, it runs a copy of the model and continuously corrects that copy using the **innovation** —
the difference between the measured and predicted output — scaled by an observer gain `L`.

## Why it matters (embedded)
State-feedback controllers (LQR, pole placement) need the *full* state vector, but sensors usually
measure only a few outputs. The Luenberger observer reconstructs the unmeasured states from the
measured ones using nothing but arithmetic — no extra sensors. It is the deterministic, lower-cost
cousin of the Kalman filter (no covariance propagation).

## How it works (intuition)
Run the model open-loop and it drifts from reality. Feed back the output error through `L` and the
estimate is pulled toward the true state. The estimation error obeys `e[k+1] = (A−L·C)e[k]`, so
choosing `L` to place the eigenvalues of `A−L·C` inside the unit circle makes the error decay.
**Ackermann's formula** computes the `L` that puts those poles exactly where you ask — as long as
the system is observable.

## Key parameters
- **plant (A, B, C, D)** — the state-space model being observed.
- **observer gain L** — placed via Ackermann's formula from the desired observer poles.
- **desired poles** — set 2–5× faster than the controller poles; faster means quicker convergence
  but more noise sensitivity.

## Reference
D. G. Luenberger, "An Introduction to Observers," *IEEE Trans. Automatic Control*, 16(6), 1971;
Ackermann's formula (Franklin, Powell, Emami-Naeini).

## See also
`Lqr` / `StateFeedbackController` (consumers of the estimate); `KalmanFilter` (stochastic
counterpart); `ControllabilityObservability` (verify observability before designing `L`).
