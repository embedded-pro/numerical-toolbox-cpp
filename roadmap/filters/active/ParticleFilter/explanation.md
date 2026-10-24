# Particle Filter — Overview

## What it is
A state estimator that represents its belief as a fixed set of samples ("particles") with weights,
instead of a single mean and covariance. Each particle is moved by the motion model, re-weighted by how
well it explains the measurement, and the set is periodically resampled to concentrate on the likely
states.

## Why it matters (embedded)
Kalman filters assume a single Gaussian belief. Robot localization in a symmetric building, terrain-aided
navigation and fault detection have several competing hypotheses or strongly non-linear sensors, and a
particle filter handles those directly. With a compile-time particle count it runs in fixed memory and
fixed time per step, and with the random numbers supplied by the caller it is fully reproducible.

## How it works (intuition)
Every particle is a guess of the state. The prediction step moves each guess as the system would move,
with the caller adding random disturbances. The update step scores each guess against the measurement,
and the score becomes its weight. After a few updates most weight sits on a few particles, which wastes
the rest; the effective sample size detects this. Resampling then draws a new, equally weighted set in
which good particles are duplicated and poor ones disappear. The systematic scheme uses one random
number and a comb of evenly spaced pointers, which keeps the extra randomness small.

## Key parameters
- **number of particles** — accuracy versus time and memory (compile-time).
- **resampling threshold** — fraction of the particle count below which the effective sample size triggers resampling.
- **motion model and likelihood** — supplied by the caller.

## Reference
N. J. Gordon, D. J. Salmond, A. F. M. Smith, "Novel approach to nonlinear/non-Gaussian Bayesian state
estimation," *IEE Proc. F*, 140(2), 1993; A. Doucet, A. M. Johansen, "A tutorial on particle filtering and
smoothing: fifteen years later," in *Handbook of Nonlinear Filtering* (2009).

## See also
`ExtendedKalmanFilter`, `UnscentedKalmanFilter` (Gaussian alternatives); robotics-toolbox-cpp's Monte Carlo
localization (E4).
