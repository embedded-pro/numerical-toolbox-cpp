# Residual-Form Kalman Update with χ² Gating

## Overview & Motivation

A classical Kalman filter owns its state and its measurement model: it predicts the measurement $h(x)$, subtracts it from $z$ and corrects. Many estimators cannot work that way.

- A **bearing or heading** residual must be wrapped to $(-\pi, \pi]$. Subtracting $z - h(x)$ directly turns a $2^\circ$ error near $\pm 180^\circ$ into a $358^\circ$ one.
- A **landmark** filter picks a different measurement model for every observation.
- A robust filter must **reject outliers** and **associate** measurements before applying them, using the innovation covariance that a self-contained filter keeps hidden.

The residual-form update is a stateless building block for these cases. The caller owns the state, forms the residual $r = z \ominus h(x)$ in whatever way its geometry needs, and passes the Jacobian and noise of the chosen model.
The update returns the **normalised innovation squared** (NIS), and either applies a Joseph-form correction or rejects the measurement at a χ² gate without touching the state.

## Mathematical Theory

### Innovation Covariance and NIS

For state covariance $P$, measurement Jacobian $H$ and measurement noise $R$:

$$S = H P H^T + R, \qquad \text{NIS} = r^T S^{-1} r$$

$S$ is factored once with Cholesky, $S = L L^T$, and every product with $S^{-1}$ uses two triangular solves. A non-positive-definite $S$ is reported as singular, and the state is left unchanged.

### χ² Gating

Under a consistent filter, the NIS follows a χ² distribution with $m$ degrees of freedom for an $m$-dimensional measurement. A measurement is rejected when its NIS exceeds the chosen quantile:

| Measurement size | 95 % gate | 99 % gate |
|------------------|-----------|-----------|
| 1                | 3.84      | 6.63      |
| 2                | 5.99      | 9.21      |

A NIS exactly at the gate is applied. The default gate is infinite, so no measurement is rejected.

### Joseph-Form Update

The gain is computed without an explicit inverse, from $S K^T = H P$:

$$K = P H^T S^{-1}, \qquad x \leftarrow x + K r$$

The covariance uses the Joseph form, which stays symmetric and positive semi-definite under rounding for any gain:

$$P \leftarrow (I - K H)\,P\,(I - K H)^T + K R K^T$$

The result is symmetrised to remove the asymmetry left by the products. The short form $(I - KH)P$ is cheaper but can lose positive-definiteness in single precision.

### Residual Convention

The residual is $z \ominus h(x)$. For angle components it is the wrapped difference, and after the update the caller re-wraps any angle in the state, since only it knows which components are angles. Independent measurements can be applied one after another, each with its own $H$ and $R$; for a linear model this equals one stacked update.

## Complexity Analysis

| Operation | Time                     | Space    | Notes                                           |
|-----------|--------------------------|----------|-------------------------------------------------|
| NIS only  | $O(N^2 M + N M^2 + M^3)$ | $O(M^2)$ | Innovation covariance and one Cholesky solve    |
| Update    | $O(N^2 M + N M^2 + M^3)$ | $O(N M)$ | One factorisation, $N$ solve pairs for the gain |

For $N$ states and $M$ measurements. The localization cases ($N \le 4$, $M \le 2$) take under 200 multiply-adds.

## Step-by-Step Walkthrough

State $x = (1, 0.5)$ with covariance $P = \begin{pmatrix} 0.5 & 0.1 \\ 0.1 & 0.3 \end{pmatrix}$, measuring the first component with noise $R = 0.2$:

1. **Scalar update**: the measurement $1.4$ gives the residual $0.4$, $S = 0.7$ and $K = (0.714, 0.143)$. The state becomes $(1.286, 0.557)$, the first variance drops to $0.143$, and the NIS is $0.229$.
2. **Vector update**: also measuring the sum $x_1 + x_2 = 1.2$ with noise $0.1$ gives the residual $(0.4, -0.3)$ and NIS $0.934$, and moves the state to $(1.098, 0.212)$.
3. **Association**: the NIS can be computed alone to score several candidate measurements before committing one.
4. **Gating**: an outlier at $4.0$ has NIS $12.86$, above the 99 % gate of $6.63$. It is rejected and the state is unchanged.
5. **Singular case**: with $P = 0$ and $R = 0$, $S$ is not positive definite. The update reports it and changes nothing.
6. **Angle residual**: a heading estimate of $3.1$ rad measured at $-3.1$ rad has a wrapped residual of $0.083$. The corrected heading wraps to $-3.117$ rad. The raw difference $-6.2$ would have dragged the estimate to $-1.86$ rad.
7. **Long runs**: after 100 updates the covariance is still exactly symmetric and positive definite.

## Pitfalls & Edge Cases

- **Unwrapped angle residuals**: the classic cause of bearing-only EKF divergence. Always wrap angle components of the residual.
- **Re-wrapping the state**: the update adds $K r$ to an angle without wrapping it. Wrap angle states afterwards.
- **Gate choice**: a tight gate rejects good measurements after a disturbance and can lock the filter out. Monitor the gating rate.
- **Correlated measurements**: sequential updates assume independent noise. Stack correlated measurements into one update.
- **Inconsistent noise**: an underestimated $R$ inflates the NIS and makes the gate reject valid data.

## Variants & Generalizations

- **Self-contained filters**: the Kalman and extended Kalman filters, which own their state and measurement model.
- **Square-root updates**: propagating a Cholesky factor of $P$ for extra robustness.
- **Information form**: accumulating $H^T R^{-1} H$ for many simultaneous measurements.
- **Multi-hypothesis association**: joint probabilistic data association and multiple-hypothesis tracking built on the NIS.

## Applications

- Wheel and gyroscope odometry fusion with heading measurements.
- Landmark-based localization with range and bearing observations.
- Target tracking with gating and nearest-neighbour association.
- Fault detection by monitoring the NIS of each sensor.

## Connections to Other Algorithms

- [Kalman Filter](KalmanFilter.md): the self-contained linear filter with the same update equations.
- [Extended Kalman Filter](ExtendedKalmanFilter.md): forms the residual itself and fixes the measurement model at construction.
- [Square-Root Kalman Filter](SquareRootKalmanFilter.md): a factored alternative for poorly conditioned problems.

## References & Further Reading

- Maybeck, P.S. (1979). *Stochastic Models, Estimation, and Control*, Vol. 1. Academic Press. Chapter 5.
- Bar-Shalom, Y., Li, X.-R. and Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley. Chapter 5.
- Bucy, R.S. and Joseph, P.D. (1968). *Filtering for Stochastic Processes with Applications to Guidance*. Interscience.
