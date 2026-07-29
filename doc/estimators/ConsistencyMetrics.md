# Estimator Consistency Metrics (NEES / NIS)

## Overview & Motivation

State estimators such as the Kalman filter, Extended Kalman filter, and Unscented Kalman filter produce both a state estimate and a covariance matrix that quantifies estimation uncertainty. A filter is said to be consistent when the true errors are statistically compatible with the reported covariance — that is, the filter neither overestimates nor underestimates its own uncertainty.

Raw error metrics such as RMSE cannot distinguish a consistent filter (correct covariance) from an inconsistent one (wrong covariance that happens to produce low errors in a specific trial). Consistency metrics address this gap by normalising errors with respect to the predicted covariance and comparing the result against a chi-squared distribution.

## Mathematical Theory

### Normalised Estimation Error Squared (NEES)

Given the state error $\varepsilon_k = x_k - \hat{x}_{k|k}$ and the posterior state covariance $P_{k|k}$, the NEES at time step $k$ is

$$\varepsilon_k^{\mathsf{T}} P_{k|k}^{-1} \varepsilon_k$$

Under the hypothesis that the filter is consistent and the errors are Gaussian, this quantity is chi-squared distributed with $n_x$ degrees of freedom, where $n_x$ is the state dimension.

### Normalised Innovation Squared (NIS)

Given the innovation $\nu_k = z_k - \hat{z}_{k|k-1}$ and the innovation covariance $S_k$, the NIS at time step $k$ is

$$\nu_k^{\mathsf{T}} S_k^{-1} \nu_k$$

Under consistency, NIS follows a chi-squared distribution with $n_z$ degrees of freedom, where $n_z$ is the measurement dimension. Unlike NEES, NIS is computable without knowledge of the true state and is therefore usable in real deployments.

### Chi-Squared Confidence Gate

A two-sided 95% confidence region for a single NEES or NIS sample is

$$\chi^2_{n,\,0.025} \;\le\; \varepsilon_k^{\mathsf{T}} P_{k|k}^{-1} \varepsilon_k \;\le\; \chi^2_{n,\,0.975}$$

where $\chi^2_{n,p}$ is the $p$-th quantile of the chi-squared distribution with $n$ degrees of freedom.

### Time-Averaged Gate

Averaging NEES or NIS over $N$ independent samples gives a statistic that is chi-squared with $N \cdot n$ degrees of freedom, scaled by $1/N$. The 95% bounds for the time-averaged value are therefore

$$\frac{\chi^2_{Nn,\,0.025}}{N} \;\le\; \bar{\varepsilon} \;\le\; \frac{\chi^2_{Nn,\,0.975}}{N}$$

Averaging reduces the variance of the consistency estimate and is the standard approach in Monte-Carlo filter evaluation.

### Embedded Computation

Rather than forming $P^{-1}$ explicitly, the quadratic form $\varepsilon^{\mathsf{T}} P^{-1} \varepsilon$ is computed by solving $P z = \varepsilon$ for $z$ (via Gaussian elimination with partial pivoting) and then computing $\varepsilon^{\mathsf{T}} z$. This avoids matrix inversion, reduces floating-point operations, and is numerically more stable.

Chi-squared quantiles for degrees of freedom 1–10 at the 95% confidence level are stored in a compile-time array. Dimensions outside this range are rejected via `static_assert`.

## Complexity Analysis

| Case    | Time     | Space    | Notes                                      |
|---------|----------|----------|--------------------------------------------|
| Best    | $O(n^2)$ | $O(n^2)$ | Dominated by back-substitution in GE       |
| Average | $O(n^3)$ | $O(n^2)$ | Gaussian elimination with partial pivoting |
| Worst   | $O(n^3)$ | $O(n^2)$ | All pivots require row swaps               |

All storage is stack-allocated; $n$ is bounded at compile time by `Dim`.

## Step-by-Step Walkthrough

Consider a 2-D state ($n = 2$) with error $\varepsilon = [1, 2]^{\mathsf{T}}$ and covariance $P = \mathrm{diag}(1, 4)$.

1. Solve $P z = \varepsilon$: $z_1 = 1/1 = 1$, $z_2 = 2/4 = 0.5$.
2. Compute dot product: $\varepsilon^{\mathsf{T}} z = 1 \cdot 1 + 2 \cdot 0.5 = 2.0$.
3. Compare against $\chi^2_{2, 0.025} \approx 0.051$ and $\chi^2_{2, 0.975} \approx 7.38$.
4. Since $2.0 \in [0.051, 7.38]$, the filter is declared consistent at the 95% level.

## Pitfalls & Edge Cases

A singular covariance matrix causes the linear solve to fail; the implementation detects near-zero diagonal pivots and returns `std::nullopt`. Callers must check the optional before using the value.

A single sample NEES or NIS value has high variance under chi-squared; a filter may appear inconsistent simply due to random variation. Time-averaging over many Monte-Carlo runs is the statistically correct procedure.

NEES requires ground-truth state access and is therefore only applicable in simulation. NIS is the consistency monitor of choice for deployed systems.

## Variants & Generalizations

The average NEES over a Monte-Carlo ensemble of $M$ runs and $K$ time steps yields $M \cdot K$ samples; consistency must hold jointly across the ensemble, not just per run. Extensions to non-Gaussian cases (e.g., particle filters) use empirical quantiles rather than chi-squared bounds.

## Applications

- Monte-Carlo evaluation of Kalman-family filters during design and tuning.
- Online innovation gating in Kalman filters (rejecting outlier measurements whose NIS exceeds the threshold).
- Diagnosing process-noise or measurement-noise mis-specification.
- Formal consistency testing as part of filter validation before deployment.

## Connections to Other Algorithms

NEES and NIS are statistical companions to the standard Kalman filter update step. They depend on the covariance propagation produced by the `filters/active` family (KF, EKF, UKF). The linear solve reuses `solvers::GaussianElimination`, and the state-error representation aligns with `estimators::EstimationMetrics`.

## References & Further Reading

- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001. Chapter 5 (Estimation Consistency).
- T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017. Chapter 9.
- S. Julier, J. Uhlmann, "A New Extension of the Kalman Filter to Nonlinear Systems," *Proc. SPIE*, 1997.
