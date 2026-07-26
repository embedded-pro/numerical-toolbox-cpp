# Complementary Filter — Overview

## What it is
A minimal sensor-fusion filter that blends a **fast but biased** sensor (a gyroscope, integrated to
an angle) with a **slow but drift-free** sensor (an accelerometer or magnetometer that measures the
angle directly): `θ = α·(θ + ω·Ts) + (1−α)·θ_meas`.

## Why it matters (embedded)
It is the default low-cost tilt/attitude estimator on virtually every IMU project. For two
multiplies and two adds it removes gyro drift and accelerometer noise simultaneously — no matrices,
no covariance, no tuning of a full Kalman filter. On an 8/16-bit MCU it is often the *only*
affordable fusion option.

## How it works (intuition)
Think of it in the frequency domain: the gyro path is a **high-pass** filter (great at high
frequency, but its integrated bias drifts at DC) and the direct-angle path is a **low-pass** filter
(accurate at DC, noisy at high frequency). Choosing weights that sum to one makes the two transfer
functions **complement** each other — `HP(s) + LP(s) = 1` — so the crossover frequency `1/τ` is the
single design knob. Below it you trust the accel/mag; above it you trust the gyro.

## Key parameters
- **α (blend weight)** `∈ [0, 1]` — how much to trust the integrated fast sensor.
- **τ (crossover time constant)** — sets the split via `α = τ/(τ+Ts)`.
- **Ts** — sample period.
- **wrap flag** — enable shortest-arc blending when the state is a heading angle.

## Reference
W. T. Higgins, "A Comparison of Complementary and Kalman Filtering," *IEEE Trans. Aerospace and
Electronic Systems*, 11(3), 1975.

## See also
`AhrsMadgwickMahony` (3-D quaternion generalization with bias estimation),
`ExponentialMovingAverage` (the low-pass path in isolation), `AlphaBetaFilter` (fixed-gain tracking).
