# Alpha-Beta / Alpha-Beta-Gamma Filter — Overview

## What it is
A fixed-gain predictor-corrector that tracks a target's **position and velocity** (and, in the
α-β-γ variant, **acceleration**). It has the same predict/update shape as a Kalman filter, but the
gains `α`, `β`, (`γ`) are **constants** instead of being recomputed from a covariance each step.

## Why it matters (embedded)
It delivers most of the benefit of a steady-state Kalman filter at a tiny fraction of the cost:
no matrix inverse, no covariance propagation, just a few multiply-adds per sample. That makes it
the go-to smoother/tracker for radar, ranging, encoders, and any sensor where you need a clean
position **and** a velocity estimate inside a fast control loop.

## How it works (intuition)
Each step **predicts** where the target should be using simple constant-velocity (or
constant-acceleration) kinematics, then **corrects** that prediction by a fraction of the
measurement residual. `α` controls how hard the position is pulled toward the measurement; `β`
does the same for velocity; `γ` for acceleration. Large gains → fast, noisy; small gains →
smooth, laggy. Kalata's *tracking index* ties the gains to the ratio of process noise to
measurement noise so a single scalar picks the whole set.

## Key parameters
- **α** — position gain, `∈ (0, 1)`.
- **β** — velocity gain; stability needs `0 < β < 4 − 2α`.
- **γ** — acceleration gain (α-β-γ only), for maneuvering targets.
- **Ts** — sample period, links residual to velocity/acceleration units.
- **tracking index λ** — one knob that generates critically-damped gains.

## Reference
P. Kalata, "The tracking index: A generalized parameter for α-β and α-β-γ target trackers,"
*IEEE Trans. Aerospace and Electronic Systems*, 20(2), 1984.

## See also
`KalmanFilter` (adaptive-gain generalization), `ComplementaryFilter` (frequency-domain fusion),
`ExponentialMovingAverage` (position-only smoothing).
