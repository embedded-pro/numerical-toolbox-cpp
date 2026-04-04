# Expectation-Maximization for Kalman Filter Parameters

## Overview & Motivation

A Kalman filter requires four parameter matrices — the state-transition matrix $F$, measurement matrix $H$, process-noise covariance $Q$, and measurement-noise covariance $R$ — to be known *a priori*.  In many real systems these parameters are unknown or only approximately known.

The **Expectation-Maximization (EM) algorithm** for linear Gaussian state-space models (Shumway & Stoffer, 1982; 2000) treats the latent state sequence $x_{1:T}$ as missing data and iteratively finds the maximum-likelihood parameter estimate from a batch of observations $z_{1:T}$.  Each iteration of EM is guaranteed to be non-decreasing in log-likelihood, providing a principled convergence criterion.

This implementation follows the Shumway-Stoffer formulation and operates exclusively in `float` because typical dynamics (forces, velocities, torques) exceed the $[-1, 1]$ range of Q15/Q31 fixed-point formats.

## Mathematical Theory

### State-Space Model

$$
x_{t+1} = F x_t + w_t, \quad w_t \sim \mathcal{N}(0, Q)
$$

$$
z_t = H x_t + v_t, \quad v_t \sim \mathcal{N}(0, R)
$$

with $x_0 \sim \mathcal{N}(\mu_0, P_0)$ and $T$ observations $z_0, \ldots, z_{T-1}$.

### EM Overview

EM alternates between:

- **E-step**: compute expected sufficient statistics of $x_{1:T}$ given current parameters and all observations, using the RTS smoother.
- **M-step**: analytically maximize the expected complete-data log-likelihood over parameters.

### E-Step (RTS Smoother)

Run `KalmanSmoother::Smooth()` to obtain, for each $t$:

| Quantity | Notation | Description |
|----------|----------|-------------|
| $\hat{x}_{t\|T}$ | `smoothedMeans[t]` | Smoothed state mean |
| $P_{t\|T}$ | `smoothedCovariances[t]` | Smoothed state covariance |
| $P_{t,t-1\|T}$ | `lagCrossCovariances[t]` | Lag-1 cross-covariance (index 0 undefined) |

### Sufficient Statistics

From the smoother output, accumulate six matrices:

$$
A = \sum_{t=1}^{T-1} P_{t,t-1|T}
\qquad
B = \sum_{t=0}^{T-2} \hat{x}_{t|T}\hat{x}_{t|T}^\top + P_{t|T}
$$

$$
C = \sum_{t=1}^{T-1} \hat{x}_{t|T}\hat{x}_{t|T}^\top + P_{t|T}
\qquad
D = \sum_{t=0}^{T-1} \hat{x}_{t|T}\hat{x}_{t|T}^\top + P_{t|T}
$$

$$
W = \sum_{t=0}^{T-1} z_t \hat{x}_{t|T}^\top
\qquad
\Sigma_{zz} = \sum_{t=0}^{T-1} z_t z_t^\top
$$

### M-Step (Closed-Form Updates)

$$
F \leftarrow A B^{-1}
$$

$$
Q \leftarrow \frac{C - F A^\top}{T - 1}, \quad Q \leftarrow \tfrac{1}{2}(Q + Q^\top) + \varepsilon I
$$

$$
H \leftarrow W D^{-1}
$$

$$
R \leftarrow \frac{\Sigma_{zz} - H W^\top}{T}, \quad R \leftarrow \tfrac{1}{2}(R + R^\top) + \varepsilon I
$$

$$
\mu_0 \leftarrow \hat{x}_{0|T}, \quad P_0 \leftarrow P_{0|T}
$$

The symmetrisation and ridge term $\varepsilon I$ ($\varepsilon = 10^{-6}$) enforce that $Q$ and $R$ remain symmetric positive definite, preventing numerical collapse.

Matrix "division" — e.g. $A B^{-1}$ — is implemented as `SolveSystem(B, A^T)^T` (Gaussian elimination) to avoid explicit matrix inversion.

### Convergence

After each EM iteration, compute the log-likelihood $\ell(\theta)$ from the RTS smoother.  Convergence is declared when

$$
|\ell^{(k)} - \ell^{(k-1)}| < \delta
$$

for a user-supplied tolerance $\delta$.

## Complexity Analysis

| Case  | Time per iteration    | Space (stack)          | Notes |
|-------|-----------------------|------------------------|-------|
| All   | $O(T \cdot N^3)$      | $O(T \cdot N^2)$       | Dominated by RTS smoother forward/backward |

All storage is stack-allocated inside `KalmanSmoother` (owned by `ExpectationMaximization`) and in local M-step temporaries.

## Step-by-Step Walkthrough

Consider $N=1$, $M=1$, $T=3$, observations $z=[0.5, 0.3, 0.8]$, initial guess $F=1, H=1, Q=0.5, R=1, \mu_0=0, P_0=1$.

**Iteration 1:**

1. E-step via RTS smoother: obtain $\hat{x}_{0|3}, \hat{x}_{1|3}, \hat{x}_{2|3}$ and corresponding covariances and lag cross-covariances.
2. Accumulate scalars:
   - $A = P_{2,1|3} + P_{3,2|3}$ (indices 2 and 3 in the 1-indexed convention).  *(In the implementation indices 1..T-1 are used.)*
   - $B, C, D, W, \Sigma_{zz}$ as sums over the smoothed moments.
3. M-step: $F \leftarrow A/B$, $Q \leftarrow (C - F\cdot A)/(T-1)$, etc.
4. Evaluate new log-likelihood $\ell^{(1)}$.

Repeat until $|\ell^{(k)} - \ell^{(k-1)}| < \delta$ or maximum iterations reached.

## Pitfalls & Edge Cases

- **Local optima**: EM converges to a local maximum of the likelihood, not necessarily the global maximum.  The quality of the solution depends heavily on the initial parameter guess.
- **Short sequences ($T \ll N$)**: With fewer observations than state dimensions, some sufficient-statistic matrices become rank-deficient and `SolveSystem` may produce inaccurate results.  Prefer $T \gg N$.
- **$Q$ or $R$ collapse**: Without the $\varepsilon I$ regularisation, a maximally tight model (zero-noise) can collapse $Q$ or $R$ to zero, making the likelihood degenerate.  If this happens, increase the regularisation or constrain the parameters.
- **Float only**: `ExpectationMaximization` has no template type parameter.  Dynamics values typically exceed the $[-1, 1]$ range of Q15/Q31.  Scale your state variables into $[-1, 1]$ and use a float wrapper if fixed-point EM is required.
- **Re-entrant smoother**: `ExpectationMaximization` owns a `KalmanSmoother` instance.  The smoother state is overwritten on every call to `Run()`.  Do not share an `ExpectationMaximization` instance across threads.

## Variants & Generalizations

- **EM for EKF/UKF**: Replace the RTS smoother with an extended or unscented smoother for nonlinear models.  The M-step remains identical.
- **Constrained EM**: Fix some parameters (e.g., $H$ is known) by simply not updating them in the M-step.
- **Online (incremental) EM**: Accumulate sufficient statistics incrementally to handle streaming data.
- **Structural constraints on $F$**: If $F$ has a known parametric form (e.g., a companion matrix), the M-step can be modified to enforce the structure.

## Applications

- **System identification**: Learn the dynamics $F$ and noise statistics $Q$, $R$ of an unknown linear system from input-output data.
- **Sensor calibration**: Identify measurement noise covariance $R$ from reference measurements.
- **Hyperparameter tuning for KF**: Replace hand-tuned $Q$, $R$ with data-driven estimates before deploying a real-time Kalman filter.
- **Time-series model selection**: Compare log-likelihoods of different state-space structures to select model order.

## Connections to Other Algorithms

- **`filters::KalmanSmoother`**: Provides the complete E-step.  `ExpectationMaximization` owns a `KalmanSmoother` instance internally.
- **`filters::KalmanFilter`**: After EM converges, pass the resulting `EmParameters` to `KalmanFilter::SetStateTransition()`, `SetProcessNoise()`, and `SetMeasurementNoise()` for real-time deployment.
- **`solvers::GaussianElimination`**: Used internally (via `KalmanSmoother`) for numerically stable matrix solves in both the smoother and the M-step.

## References & Further Reading

- Shumway, R. H., & Stoffer, D. S. (1982). An approach to time series smoothing and forecasting using the EM algorithm. *Journal of Time Series Analysis*, 3(4), 253–264.
- Shumway, R. H., & Stoffer, D. S. (2000). *Time Series Analysis and Its Applications*. Springer. (Section 6.3)
- Dempster, A. P., Laird, N. M., & Rubin, D. B. (1977). Maximum likelihood from incomplete data via the EM algorithm. *Journal of the Royal Statistical Society, Series B*, 39(1), 1–38.
- Ghahramani, Z., & Hinton, G. E. (1996). Parameter estimation for linear dynamical systems. Technical Report CRG-TR-96-2, University of Toronto.
