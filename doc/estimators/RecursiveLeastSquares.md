# Recursive Least Squares (RLS)

## Overview & Motivation

Recursive Least Squares (RLS) is an online parameter estimation algorithm that incrementally updates a least-squares fit as new data arrives, without reprocessing past observations. It converges faster than gradient-descent methods (e.g., LMS) and is well-suited for real-time system identification and adaptive filtering on embedded systems.

## Mathematical Theory

### Problem Setting

Given a linear model $y = \mathbf{x}^T \boldsymbol{\theta} + \epsilon$, where $\mathbf{x} \in \mathbb{R}^p$ is the regressor, $\boldsymbol{\theta} \in \mathbb{R}^p$ are the unknown parameters, and $\epsilon$ is noise, RLS minimizes the exponentially weighted cost:

$$J(\boldsymbol{\theta}) = \sum_{k=1}^{n} \lambda^{n-k} (y_k - \mathbf{x}_k^T \boldsymbol{\theta})^2$$

where $\lambda \in (0, 1]$ is the forgetting factor.

### Update Equations

Given a new observation $(\mathbf{x}, y)$:

1. **Gain vector**: $\mathbf{g} = P \mathbf{x}$
2. **Denominator**: $d = \lambda + \mathbf{x}^T P \mathbf{x}$
3. **Innovation (pre-update error)**: $e = y - \mathbf{x}^T \hat{\boldsymbol{\theta}}$
4. **Parameter update**: $\hat{\boldsymbol{\theta}} \leftarrow \hat{\boldsymbol{\theta}} + \frac{\mathbf{g} \cdot e}{d}$
5. **Covariance update**: $P \leftarrow \frac{1}{\lambda}\left(P - \frac{\mathbf{g} \mathbf{g}^T}{d}\right)$

### Forgetting Factor

- $\lambda = 1$: Full memory — equivalent to batch least squares with all past data equally weighted.
- $\lambda < 1$: Exponential forgetting — recent data is weighted more heavily, enabling tracking of time-varying systems.
- Typical values: $0.95 \leq \lambda \leq 1.0$.

### Convergence Evaluation

The implementation provides convergence diagnostics via `EstimationMetrics`:
- **Innovation**: Pre-update prediction error magnitude
- **Residual**: Post-update prediction error magnitude
- **Uncertainty**: Trace of the covariance matrix $P$ (overall parameter uncertainty)

## Complexity Analysis

| Case    | Time     | Space    | Notes                                  |
|---------|----------|----------|----------------------------------------|
| Average | $O(p^2)$ | $O(p^2)$ | Matrix-vector products + outer product |

Per-update cost is $O(p^2)$ for $p$ parameters, dominated by the covariance update. This is significantly cheaper than the $O(p^3)$ batch solution via matrix inversion.

## Step-by-Step Walkthrough

Estimating the slope of $y = 2x$ with RLS ($p = 2$: bias + slope, $\lambda = 1$):

| Step | $\mathbf{x}$ | $y$ | $\hat{\theta}_0$ (bias) | $\hat{\theta}_1$ (slope) | Innovation |
|------|--------------|-----|-------------------------|--------------------------|------------|
| Init | —            | —   | 0.0                     | 0.0                      | —          |
| 1    | [1, 1]       | 2   | 1.0                     | 1.0                      | 2.0        |
| 2    | [1, 2]       | 4   | 0.0                     | 2.0                      | 1.0        |
| 3    | [1, 3]       | 6   | 0.0                     | 2.0                      | 0.0        |

Convergence is achieved in 2 steps for this noiseless system.

## Pitfalls & Edge Cases

- **Covariance blow-up**: With $\lambda < 1$ and insufficient excitation, the covariance matrix $P$ can grow unbounded. Monitor the uncertainty metric and consider covariance resetting.
- **Numerical conditioning**: The covariance update can lose symmetry and positive-definiteness due to floating-point errors. For long-running applications, consider using the Joseph form: $P \leftarrow (I - K\mathbf{x}^T)P(I - K\mathbf{x}^T)^T + K K^T / \lambda$.
- **Initial covariance**: A large initial $P$ (e.g., $\alpha I$ with $\alpha = 100$) gives fast initial convergence but higher initial sensitivity. A small $P$ gives conservative updates.
- **Regressor scaling**: Large differences in regressor magnitudes can cause numerical issues. Normalize inputs when possible.

## Variants & Generalizations

- **Weighted RLS**: Use a general weighting matrix instead of scalar forgetting.
- **QR-decomposition RLS**: Better numerical stability by avoiding explicit covariance updates.
- **Kalman filter**: RLS is equivalent to a steady-state Kalman filter for the linear regression problem.
- **Regularized RLS**: Add a regularization term $\mu \|\boldsymbol{\theta}\|^2$ to prevent overfitting.

## Applications

- Online system identification for adaptive control
- Adaptive noise cancellation
- Channel equalization in communications
- Real-time parameter tracking in engine/motor control
- Sensor calibration

## Connections to Other Algorithms

- **Linear Regression**: Batch (offline) version of the same problem. Linear regression solves via matrix inversion; RLS updates incrementally.
- **Yule-Walker**: Both estimate AR model parameters, but Yule-Walker uses autocorrelation statistics (offline) while RLS processes samples sequentially.
- **Kalman Filter**: RLS is a special case of the Kalman filter where the state transition is identity and the process noise is zero.
- **Gradient Descent**: LMS/gradient descent is a simpler adaptive algorithm with $O(p)$ cost but slower convergence compared to RLS's $O(p^2)$.

## References & Further Reading

- Haykin, S., *Adaptive Filter Theory*, 5th ed., Pearson, 2014 — Chapter 9.
- Ljung, L., *System Identification: Theory for the User*, 2nd ed., Prentice Hall, 1999 — Chapter 11.
- Åström, K.J. and Wittenmark, B., *Adaptive Control*, 2nd ed., Dover, 2008 — Chapter 2.
