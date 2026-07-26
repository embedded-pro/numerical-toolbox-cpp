# Complementary Filter

## Overview & Motivation

Real-time attitude and heading estimation requires fusing two fundamentally different sensor
modalities: a **gyroscope** that integrates angular rate to produce a short-term angle estimate
(fast response, low noise, but subject to drift) and an **accelerometer or magnetometer** that
reads the angle directly (accurate at rest, noisy during motion, slow dynamics). Neither sensor
alone is sufficient. The complementary filter solves the fusion problem with two multiplies
and two adds per sample, making it the default tilt estimator on virtually every IMU-based
embedded project where a full Kalman filter is unaffordable.

## Mathematical Theory

### Frequency-Domain Complement

The two sensor paths are complementary in the transfer-function sense: the gyro path acts as a
**first-order high-pass filter** and the direct-angle path acts as a **first-order low-pass filter**
sharing the same crossover frequency $\omega_c = 1/\tau$. Their sum is identically unity for all
frequencies:

$$H_{HP}(s) + H_{LP}(s) = 1$$

This ensures that no frequency content is amplified or attenuated by the fusion itself.

### Discrete-Time Update Equation

Given the fused angle $\theta_k$, gyro rate $\omega_k$, accelerometer angle $\theta_{acc,k}$,
sample period $T_s$, and blend weight $\alpha \in [0,1]$:

$$\theta_{k+1} = \alpha\,(\theta_k + \omega_k\,T_s) + (1-\alpha)\,\theta_{acc,k}$$

The term $\theta_k + \omega_k T_s$ is the **high-pass path** (integration of the fast sensor),
and $\theta_{acc,k}$ is the **low-pass path** (direct measurement from the slow sensor).

### Design Parameter

The single tuning knob is the crossover time constant $\tau$, which maps to $\alpha$ via:

$$\alpha = \frac{\tau}{\tau + T_s}$$

Below $1/\tau$ the filter trusts the accelerometer; above it, the gyroscope. Typical embedded
values are $\tau = 0.5$–$2$ s, corresponding to $\alpha \approx 0.98$ at $T_s = 10$ ms.

### Heading Wrap (Shortest-Arc Blend)

When the state is a heading angle in $(-\pi, \pi]$, a naive linear blend can jump by $2\pi$ near
the seam. Instead the blend is performed along the shortest arc:

$$\delta = \mathrm{WrapToPi}(\theta_{acc} - \hat{\theta})$$
$$\theta_{k+1} = \mathrm{WrapToPi}\!\left(\hat{\theta} + (1-\alpha)\,\delta\right)$$

where $\mathrm{WrapToPi}(x) = \bigl((x + \pi) \bmod 2\pi\bigr) - \pi$.

## Complexity Analysis

| Operation | Time   | Space  | Notes                                      |
|-----------|--------|--------|--------------------------------------------|
| Update    | $O(1)$ | $O(1)$ | 2 multiplies, 2 adds; 1 fmod when wrapping |
| Reset     | $O(1)$ | $O(1)$ | Single state write                         |

Total storage: one angle word plus two constant coefficients.

## Step-by-Step Walkthrough

**Setup:** $\alpha = 0.98$, $T_s = 10$ ms, initial angle $= 0$.

**Sample 1:** gyro rate $\omega = 10$ deg/s $= 0.1745$ rad/s, accel reads $\theta_{acc} = 0.01$ rad.

$$\hat{\theta} = 0 + 0.1745 \times 0.01 = 0.001745 \text{ rad} \quad (\text{gyro path})$$
$$\theta_1 = 0.98 \times 0.001745 + 0.02 \times 0.01 = 0.001710 + 0.000200 = 0.001910 \text{ rad}$$

**After many samples with zero rate and accel $= 0.2$ rad:** the low-pass term accumulates
and $\theta \to 0.2$ rad as $(1-\alpha)^n \to 0$.

## Pitfalls & Edge Cases

- **Alpha at 1:** the gyro path integrates without bound; any constant bias drifts the angle
  indefinitely. This is intentional (gyro-only mode) but must be avoided in production.
- **Alpha at 0:** the output equals the accelerometer reading at every step; the gyro is ignored.
- **Heading wrap:** linear blending without shortest-arc correction produces a $2\pi$ jump when
  the heading crosses $\pm\pi$. Always enable the wrap mode for heading estimation.
- **Gyro bias:** the filter has no bias estimator. A constant gyro bias produces a bounded
  steady-state error of approximately $\text{bias} \cdot T_s \cdot \alpha / (1-\alpha)$.
  Pre-subtract a calibrated bias before calling Update.
- **Accelerometer noise during dynamics:** the accel path is unreliable when linear acceleration
  is present (non-gravitational). Reduce $(1-\alpha)$ or temporarily freeze the accel correction.

## Variants & Generalizations

| Variant                    | Key Difference                                                        |
|----------------------------|-----------------------------------------------------------------------|
| **Mahony filter**          | 3-D quaternion formulation with integral gyro-bias estimator          |
| **Madgwick filter**        | Gradient-descent quaternion fusion; no linearisation                  |
| **Alpha-Beta filter**      | Fixed-gain tracking without a slow sensor; pure high-pass integration |
| **Kalman filter**          | Optimal (minimum-variance) fusion; requires noise covariance tuning   |
| **Two-step complementary** | Separate pitch/roll from heading; common on 6-DOF IMUs                |

## Applications

- **IMU tilt estimation** — Roll and pitch from a 6-axis MEMS sensor at low computational cost.
- **Heading fusion** — Combining gyro yaw rate with magnetometer heading.
- **Servo/motor feedback** — Fusing encoder velocity with potentiometer position.
- **Altitude hold** — Mixing barometer (low-pass) with accelerometer integration (high-pass).

## Connections to Other Algorithms

| Algorithm                                                            | Relationship                                                             |
|----------------------------------------------------------------------|--------------------------------------------------------------------------|
| [Exponential Moving Average](../passive/ExponentialMovingAverage.md) | The low-pass path in isolation; $\alpha_{EMA} = 1-\alpha_{CF}$           |
| [Alpha-Beta Filter](AlphaBetaFilter.md)                              | Complementary filter without a slow-sensor reference; fixed-gain tracker |
| [AHRS Madgwick/Mahony](AhrsMadgwickMahony.md)                        | 3-D quaternion generalization with gyro-bias estimation                  |
| [Kalman Filter](KalmanFilter.md)                                     | Statistically optimal generalization requiring $Q$ and $R$ tuning        |

## References & Further Reading

- W. T. Higgins, "A Comparison of Complementary and Kalman Filtering," *IEEE Transactions on
  Aerospace and Electronic Systems*, 11(3), pp. 321–325, 1975.
- S. Madgwick, "An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays,"
  Technical Report, University of Bristol, 2010.
- R. Mahony, T. Hamel, and J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special
  Orthogonal Group," *IEEE Transactions on Automatic Control*, 53(5), pp. 1203–1218, 2008.
