# Stateful Step Optimisers (SGD / Adam)

## Overview & Motivation

On-device neural network training and online system identification require an optimiser that accepts one externally computed gradient per call and maintains its own state between calls. Unlike batch optimisers that own the full objective, a **step optimiser** separates the gradient source from the update rule, enabling training loops where back-propagation runs elsewhere.

Two industry-standard rules are provided: **Stochastic Gradient Descent** (SGD) with optional momentum or Nesterov lookahead, and the **Adam** (Adaptive Moment Estimation) optimiser with bias correction.

## Mathematical Theory

### SGD

Vanilla SGD applies the raw gradient:

$$\theta_{t+1} = \theta_t - \eta \, g_t$$

where $g_t = \nabla_\theta \mathcal{L}(\theta_t)$ and $\eta$ is the learning rate.

**Momentum** accumulates a velocity $v$, smoothing oscillations and accelerating progress in low-curvature directions:

$$v_{t+1} = \beta \, v_t + g_t, \qquad \theta_{t+1} = \theta_t - \eta \, v_{t+1}$$

with $\beta \in [0, 1)$ the momentum coefficient (typically $0.9$).

**Nesterov momentum** incorporates a lookahead correction, replacing the plain velocity step with:

$$\theta_{t+1} = \theta_t - \eta \, \bigl(g_t + \beta \, v_{t+1}\bigr)$$

This form evaluates the effective update at the anticipated next position, yielding faster convergence on smooth convex objectives.

### Adam

Adam maintains exponential moving averages of the gradient (first moment $m$) and the squared gradient (second moment $v$):

$$m_{t+1} = \beta_1 \, m_t + (1 - \beta_1) \, g_t$$

$$v_{t+1} = \beta_2 \, v_t + (1 - \beta_2) \, g_t^2 \quad (\text{element-wise})$$

Both estimates are biased toward zero at initialisation. **Bias correction** removes this bias:

$$\hat{m}_{t+1} = \frac{m_{t+1}}{1 - \beta_1^{t+1}}, \qquad \hat{v}_{t+1} = \frac{v_{t+1}}{1 - \beta_2^{t+1}}$$

The parameter update normalises the corrected first moment by the square root of the corrected second moment, providing a per-parameter adaptive step size:

$$\theta_{t+1} = \theta_t - \eta \, \frac{\hat{m}_{t+1}}{\sqrt{\hat{v}_{t+1}} + \varepsilon}$$

Typical hyper-parameters: $\eta = 10^{-3}$, $\beta_1 = 0.9$, $\beta_2 = 0.999$, $\varepsilon = 10^{-8}$.

## Complexity Analysis

| Algorithm | Time per step | Extra state (floats) | Notes                                                               |
|-----------|---------------|----------------------|---------------------------------------------------------------------|
| SGD       | $O(N)$        | $N$                  | One velocity vector (equals the gradient when momentum is 0)        |
| Adam      | $O(N)$        | $2N + 2$             | Two moment vectors plus the running powers $\beta_1^t$, $\beta_2^t$ |

$N$ is the number of parameters. All operations are in-place; no heap allocation is required.

## Step-by-Step Walkthrough

**SGD with momentum** on $\mathcal{L}(\theta) = \frac{1}{2}\|\theta\|^2$, $N=1$, $\theta_0 = 1$, $\eta = 0.1$, $\beta = 0.9$:

| $t$ | $g_t = \theta_t$ | $v_t = 0.9 v_{t-1} + g_t$ | $\theta_{t+1} = \theta_t - 0.1 v_t$ |
|-----|------------------|---------------------------|-------------------------------------|
| 1   | 1.000            | 1.000                     | 0.900                               |
| 2   | 0.900            | 1.800                     | 0.720                               |
| 3   | 0.720            | 2.340                     | 0.486                               |

**Adam** on the same objective, $\beta_1 = 0.9$, $\beta_2 = 0.999$, $\varepsilon = 10^{-8}$, $\theta_0 = 0$, $g_1 = 1$:

| Quantity    | Value                  |
|-------------|------------------------|
| $m_1$       | $0.1$                  |
| $v_1$       | $0.001$                |
| $\hat{m}_1$ | $1.0$                  |
| $\hat{v}_1$ | $1.0$                  |
| $\theta_1$  | $-\eta \approx -0.001$ |

Bias correction is the critical step: without it, $m_1/\sqrt{v_1} \approx 3.16$, giving a first step roughly $\sqrt{1000}$ times larger than the bias-corrected value.

## Pitfalls & Edge Cases

- **Learning rate too large.** SGD without momentum diverges for $\eta \geq 2/L$ on $L$-smooth losses. Momentum reduces the effective stability bound further; reduce $\eta$ or $\beta$ if oscillation is observed.
- **Adam with small $\varepsilon$.** Setting $\varepsilon$ too small causes division by near-zero when a parameter has zero gradient history, producing numerical instability. The default $10^{-8}$ is sufficient for `float`.
- **Momentum at reset.** When `Reset()` is called, the velocity (SGD) or moment estimates (Adam) return to zero. The first step after a reset behaves identically to starting from scratch, so the bias-correction denominator for Adam also restarts from $t=1$.
- **Nesterov with large $\beta$.** The lookahead correction adds $\beta \, v_{t+1}$ to the update, which can overshoot on sparse or noisy gradients. Prefer plain momentum when the gradient signal is noisy.
- **Float precision.** For Adam, the accumulated second moment $v$ can underflow toward zero for very small gradients and single-precision arithmetic. Increasing $\varepsilon$ mitigates this at the cost of less adaptivity.

## Variants & Generalizations

| Variant | Change from base                                                                               |
|---------|------------------------------------------------------------------------------------------------|
| AdaGrad | Non-decaying sum of squared gradients (no $\beta_2$ decay); aggressive learning rate shrinkage |
| RMSProp | Adam without first-moment tracking; lacks bias correction                                      |
| AdamW   | Decoupled weight-decay applied directly to $\theta$ before the gradient step                   |
| AMSGrad | Replaces $\hat{v}$ with the running maximum to guarantee monotone effective step-size          |

## Applications

- **On-device neural network training** — Updating weights after each batch of sensor data.
- **Online system identification** — Fitting model parameters in real time as measurements arrive.
- **Adaptive control** — Adjusting gain schedules or feed-forward maps without offline re-training.
- **Sensor calibration** — Minimising residual error by incrementally fitting a polynomial or affine model.

## Connections to Other Algorithms

| Component                                      | Relationship                                                                                   |
|------------------------------------------------|------------------------------------------------------------------------------------------------|
| [Gradient Descent](Optimizer.md)               | Batch counterpart; shares the learning-rate update rule but recomputes the objective each time |
| [LMS Adaptive Filter](../estimators/README.md) | Equivalent to online SGD for a linear regression model under MSE loss                          |
| [Regularization](../regularization/README.md)  | Adds a penalty gradient to $g_t$; compatible with any step optimiser                           |

## References & Further Reading

- Ruder, S., "An overview of gradient descent optimization algorithms", *arXiv:1609.04747*, 2016.
- Kingma, D.P. and Ba, J., "Adam: A Method for Stochastic Optimization", *ICLR*, 2015.
- Nesterov, Y., "A method for solving the convex programming problem with convergence rate $O(1/k^2)$", *Soviet Mathematics Doklady*, 1983.
- Goodfellow, I., Bengio, Y., and Courville, A., *Deep Learning*, Chapter 8, MIT Press, 2016.
