# Recursive Instrumental Variables (RIV)

## Overview & Motivation

Recursive least squares assumes the regressor is known exactly and only the output is noisy. When
the regressor is itself measured — a derivative taken from two noisy samples, a current read
through a noisy sensor — least squares reads the noise as a weaker dependence on that column and
biases its coefficient toward zero. The effect is regression dilution, and it does not shrink with
more data: the estimate converges, but to the wrong value.

The instrumental-variable method removes that bias without modelling the noise. It needs a second
signal per observation, the *instrument*, that moves with the true regressor but not with its
noise. A known excitation is the usual choice: the reference a controller injects, a command, or a
low-noise measurement of the same quantity. The recursive form updates the estimate sample by
sample at the cost of recursive least squares.

## Mathematical Theory

### Problem Setting

The model is $y = \mathbf{x}^T \boldsymbol{\theta} + \epsilon$, but only $\tilde{\mathbf{x}} =
\mathbf{x} + \mathbf{v}$ is observed, with $\mathbf{v}$ zero-mean noise. Least squares solves
$\sum \tilde{\mathbf{x}}\,(y - \tilde{\mathbf{x}}^T \boldsymbol{\theta}) = 0$. Because
$E[\tilde{\mathbf{x}} \tilde{\mathbf{x}}^T] = R_x + R_v$, its limit is

$$\hat{\boldsymbol{\theta}}_{LS} \to (R_x + R_v)^{-1} R_x\, \boldsymbol{\theta}$$

For a single slope this is the attenuation $\sigma_x^2 / (\sigma_x^2 + \sigma_v^2)$.

### Instrumental-Variable Estimate

Given an instrument $\mathbf{z}$ correlated with $\mathbf{x}$ and uncorrelated with $\mathbf{v}$ and
$\epsilon$, the estimate solves

$$\sum_{k} \lambda^{n-k}\, \mathbf{z}_k \left(y_k - \tilde{\mathbf{x}}_k^T \boldsymbol{\theta}\right) = 0
\quad\Rightarrow\quad
\hat{\boldsymbol{\theta}} = \Big(\sum_k \lambda^{n-k} \mathbf{z}_k \tilde{\mathbf{x}}_k^T\Big)^{-1} \sum_k \lambda^{n-k} \mathbf{z}_k y_k$$

The noise enters only through $E[\mathbf{z}\,\mathbf{v}^T] = 0$, so the estimate is consistent.

### Update Equations

With $P$ the inverse of the weighted cross-moment $\sum \lambda^{n-k} \mathbf{z}_k
\tilde{\mathbf{x}}_k^T$, a new observation $(\tilde{\mathbf{x}}, \mathbf{z}, y)$ gives:

1. **Gain vector**: $\mathbf{g} = P \mathbf{z}$
2. **Denominator**: $d = \lambda + \tilde{\mathbf{x}}^T P \mathbf{z}$
3. **Innovation**: $e = y - \tilde{\mathbf{x}}^T \hat{\boldsymbol{\theta}}$
4. **Parameter update**: $\hat{\boldsymbol{\theta}} \leftarrow \hat{\boldsymbol{\theta}} + \mathbf{g}\, e / d$
5. **Matrix update**: $P \leftarrow \frac{1}{\lambda}\left(P - \mathbf{g}\, \tilde{\mathbf{x}}^T P / d\right)$

Steps 2 and 5 are the Sherman–Morrison update of a rank-one, non-symmetric term
$\mathbf{z}\tilde{\mathbf{x}}^T$. With $\mathbf{z} = \tilde{\mathbf{x}}$ every step reduces to
recursive least squares.

### Diagnostics

The innovation, the post-update residual and the trace of $P$ are reported as for recursive least
squares. $P$ is no longer a scaled covariance and need not be symmetric; its trace still falls as
the instrument excites the parameters.

## Complexity Analysis

| Case    | Time     | Space    | Notes                                            |
|---------|----------|----------|--------------------------------------------------|
| Average | $O(p^2)$ | $O(p^2)$ | Two matrix-vector products and one outer product |

The cost per update matches recursive least squares: one more matrix-vector product, for
$\tilde{\mathbf{x}}^T P$, replaces the symmetric outer product.

## Step-by-Step Walkthrough

A single slope, $y = 2x$, observed through $\tilde{x} = x + v$ with the instrument $z = x$. The noise
alternates so that it is uncorrelated with $x$ over the four samples. $\lambda = 1$, $P_0 = 1000$,
$\hat{\theta}_0 = 0$:

| Step | $x$ | $v$  | $\tilde{x}$ | $y$ | Least squares $\hat{\theta}$ | Instrumental $\hat{\theta}$ |
|------|-----|------|-------------|-----|------------------------------|-----------------------------|
| 1    | 1   | 0.5  | 1.5         | 2   | 1.333                        | 1.332                       |
| 2    | 1   | -0.5 | 0.5         | 2   | 1.599                        | 1.999                       |
| 3    | 2   | 0.5  | 2.5         | 4   | 1.600                        | 1.714                       |
| 4    | 2   | -0.5 | 1.5         | 4   | 1.818                        | 2.000                       |

After the four samples least squares holds $2\,\sum x^2 / (\sum x^2 + \sum v^2) = 20/11$, the
attenuated slope, while the instrumental estimate is back at 2: each time the noise has averaged out
against the instrument, it is exact.

## Pitfalls & Edge Cases

- **Weak instrument**: if $\mathbf{z}$ is barely correlated with $\mathbf{x}$, the cross-moment is
  near-singular and the estimate's variance grows without bound. The instrument must excite every
  parameter.
- **Instrument shares the noise**: any correlation between $\mathbf{z}$ and $\mathbf{v}$, or
  between $\mathbf{z}$ and $\epsilon$ (a closed loop that feeds measured noise back into the
  instrument), brings the bias back in proportion.
- **Zero instrument**: an observation whose instrument is zero leaves $\hat{\boldsymbol{\theta}}$
  untouched but still divides $P$ by $\lambda$; with forgetting, a long run of such observations
  grows $P$ as in under-excited least squares. Skip those observations.
- **Variance**: the method trades bias for variance. For the same data its estimates scatter more
  than least squares, most of all when the instrument explains only part of the regressor.

## Variants & Generalizations

- **Batch IV**: the closed-form estimate above, solved once.
- **Two-stage least squares**: regress the regressor on the instruments first, then regress the
  output on the fitted regressor; with as many instruments as regressors it equals IV.
- **Bootstrap / refined IV**: build the instrument from a model simulated with the current
  estimate, as in Young's refined instrumental-variable method for transfer functions.
- **Columns as their own instruments**: columns known without noise can serve as their own
  instruments, so only the noisy columns need an external one.

## Applications

- Online identification of a winding's resistance and inductance from a derivative of measured
  current, with the injected current reference as the instrument
- Closed-loop system identification, where the reference is independent of the measurement noise
- Calibration of one sensor against another, both noisy
- Econometrics and errors-in-variables regression in general

## Connections to Other Algorithms

- **Recursive Least Squares**: the special case $\mathbf{z} = \tilde{\mathbf{x}}$; same cost, same
  forgetting factor and diagnostics.
- **Total Least Squares**: also removes errors-in-variables bias, but needs the noise ratio between
  channels and a batch SVD; IV needs an instrument instead and runs recursively.
- **Kalman filter**: a Kalman filter with the regressor noise in its model is the optimal
  alternative when that noise's covariance is known.

## References & Further Reading

- Söderström, T. and Stoica, P., *Instrumental Variable Methods for System Identification*,
  Springer, 1983.
- Ljung, L., *System Identification: Theory for the User*, 2nd ed., Prentice Hall, 1999 — §7.6 and
  §11.3.
- Young, P.C., *Recursive Estimation and Time-Series Analysis*, 2nd ed., Springer, 2011 — Chapter 6.
