# Alpha-Beta / Alpha-Beta-Gamma Filter

## Overview & Motivation

In embedded control and tracking applications, a sensor delivers a position measurement every sample period, but that measurement is corrupted by noise. A simple lowpass filter smooths the noise but cannot estimate velocity, which is needed for prediction and control. A full Kalman filter computes optimal gains but requires covariance propagation — a matrix inverse every step — which is too expensive for a fast ISR.

The alpha-beta (and its extension, alpha-beta-gamma) filter resolves this tension. It maintains a position and velocity estimate (and optionally acceleration) using only a few multiply-adds per sample. The gains are fixed constants, computed once at design time from a single scalar parameter. The result is a deterministic, constant-time predictor-corrector that delivers most of the benefit of a steady-state Kalman filter at a fraction of the cost.

## Mathematical Theory

### State Model

The filter assumes constant-velocity (order 2) or constant-acceleration (order 3) kinematics. For order 2, the state is $\mathbf{x} = [p, \dot{p}]^\top$; for order 3, $\mathbf{x} = [p, \dot{p}, \ddot{p}]^\top$.

### Predict Step

$$\hat{p}^- = \hat{p} + T_s \hat{v} + \tfrac{1}{2} T_s^2 \hat{a} \quad (\hat{a} \text{ omitted for order 2})$$
$$\hat{v}^- = \hat{v} + T_s \hat{a} \quad (\hat{a} \text{ omitted for order 2})$$
$$\hat{a}^- = \hat{a}$$

### Correct Step

Let the innovation (residual) be $r = z - \hat{p}^-$, where $z$ is the measured position. Then:

$$\hat{p} = \hat{p}^- + \alpha r$$
$$\hat{v} = \hat{v}^- + \frac{\beta}{T_s} r$$
$$\hat{a} = \hat{a}^- + \frac{2\gamma}{T_s^2} r \quad (\text{order 3 only})$$

The denominators $T_s$ and $T_s^2$ convert the dimensionless residual into velocity and acceleration corrections.

### Kalata Steady-State Design (Tracking Index)

For the order-2 case, Kalata (1984) defines the tracking index $\lambda = \frac{\sigma_w T_s^2}{\sigma_v}$, where $\sigma_w$ is process noise intensity and $\sigma_v$ is measurement noise standard deviation. The critically-damped gains are:

$$r = \frac{4 + \lambda - \sqrt{8\lambda + \lambda^2}}{4}$$
$$\alpha = 1 - r^2$$
$$\beta = 2(2 - \alpha) - 4\sqrt{1 - \alpha}$$

A single scalar $\lambda$ thus controls the smoothing/lag trade-off.

### Stability Conditions

For the order-2 filter, Simpson's triangle requires:

$$0 < \alpha < 1, \qquad 0 < \beta < 4 - 2\alpha$$

Violation of the second bound causes oscillatory divergence.

## Complexity Analysis

| Case    | Time   | Space    | Notes |
|---------|--------|----------|-------|
| Best    | $O(1)$ | $O(N)$   | $N \in \{2, 3\}$ state words plus fixed scalars |
| Average | $O(1)$ | $O(N)$   | same |
| Worst   | $O(1)$ | $O(N)$   | gains are precomputed; no covariance update |

The hot path is a handful of fused multiply-add operations: predict costs 2–4 MACs, correct costs 2–3 MACs.

## Step-by-Step Walkthrough

Consider an order-2 filter with $\alpha = 0.5$, $\beta = 0.1$, $T_s = 1.0\,\text{s}$, measuring a ramp $z[n] = 0.2n$.

| Step | $z$ | $\hat{p}^-$ | $\hat{v}^-$ | $r$ | $\hat{p}$ | $\hat{v}$ |
|------|-----|-------------|-------------|-----|-----------|-----------|
| 0 (seed) | 0.0 | — | — | — | 0.0 | 0.0 |
| 1 | 0.2 | 0.0 | 0.0 | 0.2 | 0.10 | 0.020 |
| 2 | 0.4 | 0.12 | 0.020 | 0.28 | 0.26 | 0.048 |
| … | … | … | … | … | … | … |

After several hundred steps, $\hat{v} \to 0.2$ and lag $\to 0$.

## Pitfalls & Edge Cases

- **Small $T_s$**: the corrections $\beta/T_s$ and $2\gamma/T_s^2$ grow large. Precomputing these as constants (done at construction) avoids repeated division on the hot path and flags numerical range issues early.
- **Stability boundary**: gains near $\beta = 4 - 2\alpha$ produce marginally stable responses. In practice, keep $\beta < 3 - 2\alpha$ for a margin of safety.
- **Initialization**: the first sample seeds the position; velocity and acceleration are zero. Transient overshoot on a step input decays at a rate governed by the gains.
- **Order-3 on a ramp**: the acceleration state will correctly settle near zero rather than accumulating a phantom bias, provided gains are stable.

## Variants & Generalizations

- **Order 2 ($\alpha$-$\beta$)**: tracks position and velocity; optimal for constant-velocity targets.
- **Order 3 ($\alpha$-$\beta$-$\gamma$)**: adds acceleration; suitable for maneuvering targets but requires additional tuning of $\gamma$.
- **Adaptive gains**: switching $\alpha$ between large (maneuver) and small (coast) values gives an interactive multiple-model (IMM) flavor without full Kalman complexity.
- **Steady-state Kalman**: the $\alpha$-$\beta$ filter is exactly a scalar Kalman filter whose Riccati equation has converged, making $\lambda$ the natural design parameter.

## Applications

- **Radar / ranging**: smoothing noisy range or angle measurements while estimating radial velocity.
- **Motor control**: fusing encoder position to estimate shaft velocity for a feedback loop.
- **IMU pre-filtering**: attenuating high-frequency vibration before integrating acceleration.
- **Any tight ISR**: when covariance propagation is too expensive but a plain IIR gives no velocity.

## Connections to Other Algorithms

- **KalmanFilter**: the $\alpha$-$\beta$ filter is its steady-state specialization; the full filter adapts gains to non-stationary noise.
- **ExponentialMovingAverage**: position-only smoothing — no velocity estimate, equivalent to $\beta = 0$.
- **ComplementaryFilter**: fuses two sensors in the frequency domain; similar predict/correct intuition but requires two measurement streams.

## References & Further Reading

- P. Kalata, "The tracking index: A generalized parameter for alpha-beta and alpha-beta-gamma target trackers," *IEEE Transactions on Aerospace and Electronic Systems*, 20(2), pp. 174–182, 1984.
- S. Blackman and R. Popoli, *Design and Analysis of Modern Tracking Systems*, Artech House, 1999.
- R. G. Brown and P. Y. C. Hwang, *Introduction to Random Signals and Applied Kalman Filtering*, 4th ed., Wiley, 2012.
