# Kalman Smoother (RTS Smoother)

## Overview & Motivation

The Kalman filter is a **causal** estimator: its state estimate at time $t$ uses only observations up to and including time $t$.  When the full observation sequence is available (offline/batch setting), the **Rauch-Tung-Striebel (RTS) smoother** refines every filtered estimate by incorporating future measurements.  The result is the minimum-mean-square-error (MMSE) state estimate given the *entire* sequence $z_{1:T}$.

The RTS smoother is the prerequisite for the Expectation-Maximization (EM) algorithm for Kalman Filter parameter identification: the E-step requires smoothed means, smoothed covariances, and the **lag-1 cross-covariance** $P_{t,t-1|T}$ in order to form the sufficient statistics used by the M-step.

## Mathematical Theory

### State-Space Model

$$
x_{t+1} = F x_t + w_t, \quad w_t \sim \mathcal{N}(0, Q)
$$

$$
z_t = H x_t + v_t, \quad v_t \sim \mathcal{N}(0, R)
$$

with $x_0 \sim \mathcal{N}(\mu_0, P_0)$.

### Forward Pass (Kalman Filter)

For $t = 0, 1, \ldots, T-1$:

**Initialise** (t = 0 only):

$$\hat{x}_{0|-} = \mu_0, \quad P_{0|-} = P_0$$

**Update:**

$$
S_t = H P_{t|-} H^\top + R
$$

$$
K_t = P_{t|-} H^\top S_t^{-1}
$$

$$
\hat{x}_{t|t} = \hat{x}_{t|-} + K_t (z_t - H \hat{x}_{t|-})
$$

$$
P_{t|t} = (I - K_t H) P_{t|-} (I - K_t H)^\top + K_t R K_t^\top \quad \text{(Joseph form)}
$$

**Log-likelihood contribution:**

$$
\ell_t = -\tfrac{1}{2}\bigl(\log\det S_t + (z_t - H\hat{x}_{t|-})^\top S_t^{-1} (z_t - H\hat{x}_{t|-}) + m\log 2\pi\bigr)
$$

where $m$ = `MeasurementSize`. The total is $\ell = \sum_{t=0}^{T-1} \ell_t$.

**Predict** (for $t < T-1$):

$$
\hat{x}_{t+1|-} = F \hat{x}_{t|t}, \quad P_{t+1|-} = F P_{t|t} F^\top + Q
$$

### Backward Pass (RTS Smoother)

Initialise at $t = T-1$:

$$
\hat{x}_{T-1|T} = \hat{x}_{T-1|T-1}, \quad P_{T-1|T} = P_{T-1|T-1}
$$

For $t = T-2, T-3, \ldots, 0$:

**Smoother gain:**

$$
G_t = P_{t|t} F^\top P_{t+1|t}^{-1}
$$

**Smoothed mean and covariance:**

$$
\hat{x}_{t|T} = \hat{x}_{t|t} + G_t (\hat{x}_{t+1|T} - \hat{x}_{t+1|t})
$$

$$
P_{t|T} = P_{t|t} + G_t (P_{t+1|T} - P_{t+1|t}) G_t^\top
$$

### Lag-1 Cross-Covariance

The EM M-step requires $P_{t,t-1|T} = \mathrm{Cov}(x_t, x_{t-1} \mid z_{1:T})$.

**Initialisation** (Shumway & Stoffer, 1982):

$$
P_{T-1, T-2|T} = (I - K_{T-1} H) F P_{T-2|T-2}
$$

**Recursion** for $t = T-3, \ldots, 0$:

$$
P_{t+1, t|T} = P_{t+1|t+1} G_t^\top + G_{t+1} (P_{t+2, t+1|T} - F P_{t+1|t+1}) G_t^\top
$$

Note: $P_{0,-1|T}$ is undefined; index 0 of `lagCrossCovariances` is always zero.

## Complexity Analysis

| Case  | Time           | Space                          | Notes                         |
|-------|----------------|--------------------------------|-------------------------------|
| All   | $O(T \cdot N^3)$ | $O(T \cdot N^2)$ stack         | Dominant cost: $N\times N$ solves at each step |

All storage is stack-allocated via `std::array`.  There is no heap usage.

## Step-by-Step Walkthrough

Consider a 1-D constant-position model ($N=1$, $M=1$, $T=3$) with $F=1$, $H=1$, $Q=0.1$, $R=1$, $\mu_0=0$, $P_0=1$, and observations $z = [0.5, 0.3, 0.8]$.

**Forward pass:**

| $t$ | $P_{t\|-}$ | $S_t$ | $K_t$ | $\hat{x}_{t\|t}$ | $P_{t\|t}$ |
|-----|-----------|-------|--------|------------------|-----------|
| 0   | 1.0       | 2.0   | 0.500  | 0.250            | 0.500     |
| 1   | 0.600     | 1.600 | 0.375  | 0.363            | 0.375     |
| 2   | 0.475     | 1.475 | 0.322  | 0.620            | 0.322     |

**Backward pass ($t=1$ then $t=0$):**

$t=1$: $G_1 = 0.500/0.600 = 0.833$; $\hat{x}_{1|3} = 0.363 + 0.833(0.620 - 0.363) \approx 0.577$

$t=0$: $G_0 = 0.375/0.375 = 1.000$; $\hat{x}_{0|3} = 0.250 + 1.000(0.577 - 0.363) \approx 0.464$

Notice how the smoothed $\hat{x}_{0|3}=0.464$ incorporates all three observations, whereas the filtered $\hat{x}_{0|0}=0.250$ only used the first.

## Pitfalls & Edge Cases

- **Near-singular $P_{t+1|t}$**: The smoother gain is computed via `SolveSystem` (Gaussian elimination with partial pivoting) rather than explicit matrix inversion.  If the predicted covariance becomes near-singular (possible when $Q \approx 0$), numerical accuracy degrades.  Ensure $Q$ has positive diagonal entries.
- **Joseph-form update**: The standard update $P = (I-KH)P$ is numerically unstable for finite-precision arithmetic.  This implementation uses the Joseph form to maintain positive semi-definiteness.
- **`lagCrossCovariances[0]` is always zero**: The lag-1 cross-covariance for index 0 ($P_{0,-1|T}$) is undefined.  The first valid entry is index 1 ($P_{1,0|T}$).
- **Unsigned loop guard**: The backward loop uses `std::size_t`; the break guard `if (t == 0) break` prevents unsigned wrap-around.
- **Stack usage**: For large $N$ or $T$, the five internal `std::array` members dominate stack usage.  Choose `MaxSteps` conservatively on resource-constrained targets.

## Variants & Generalizations

- **Square-root RTS smoother**: Propagates Cholesky factors instead of full covariance matrices for improved numerical conditioning.
- **Information filter smoother**: Operates in the dual (information) domain; preferable when observations are dense relative to state transitions.
- **Extended / Unscented smoother**: Replace the linear forward pass with EKF or UKF; the backward pass equations remain identical in form.

## Applications

- **EM parameter identification**: Used as the E-step by `ExpectationMaximization` to compute the sufficient statistics $\{A, B, C, D, W, \Sigma_{zz}\}$ for the M-step.
- **Offline trajectory estimation**: Position/velocity smoothing in GNSS post-processing and inertial navigation.
- **Batch signal denoising**: Any application where the full observation sequence is available before estimation begins.

## Connections to Other Algorithms

- **`filters::KalmanFilter`**: The forward pass of `KalmanSmoother` replicates the KF update equations.  The smoother is applied *after* the filter, not *instead of* it.
- **`estimators::ExpectationMaximization`**: The smoother is owned and invoked by `ExpectationMaximization` as its E-step.  Users who only need EM should interact with `ExpectationMaximization` directly.
- **`solvers::GaussianElimination`**: Used for all matrix "division" operations (smoother gain, Kalman gain) to avoid explicit matrix inversion.
- **`solvers::CholeskyDecomposition`**: Used to compute $\log \det S_t$ for the log-likelihood.

## References & Further Reading

- Rauch, H. E., Tung, F., & Striebel, C. T. (1965). Maximum likelihood estimates of linear dynamic systems. *AIAA Journal*, 3(8), 1445–1450.
- Shumway, R. H., & Stoffer, D. S. (1982). An approach to time series smoothing and forecasting using the EM algorithm. *Journal of Time Series Analysis*, 3(4), 253–264.
- Shumway, R. H., & Stoffer, D. S. (2000). *Time Series Analysis and Its Applications*. Springer. (Chapter 6)
- Särkkä, S. (2013). *Bayesian Filtering and Smoothing*. Cambridge University Press. (Chapter 8)
