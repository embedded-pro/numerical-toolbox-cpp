# Bayesian Optimization

## Overview & Motivation

Bayesian Optimization (BO) is a gradient-free, sample-efficient strategy for optimizing
black-box functions that are expensive to evaluate. Unlike gradient-based methods (gradient
descent, Adam), BO does not require access to derivatives. It is ideal for calibrating
numerical controllers, tuning hyperparameters, or any optimization task where each function
evaluation is costly — for example, running a full closed-loop simulation.

BO maintains a probabilistic surrogate model (typically a Gaussian Process) of the objective
function. At each iteration, it uses an acquisition function to select the next query point
that balances exploration (uncertain regions) with exploitation (promising regions). After
evaluating the true objective at that point, the surrogate is updated and the process repeats.

## Mathematical Theory

### Gaussian Process Surrogate

A Gaussian Process (GP) defines a distribution over functions:

$$f \sim \mathcal{GP}(m(\mathbf{x}), k(\mathbf{x}, \mathbf{x}'))$$

With zero mean $m = 0$ and squared-exponential (RBF) kernel:

$$k(\mathbf{x}, \mathbf{x}') = \sigma_f^2 \exp\!\left(-\frac{\|\mathbf{x} - \mathbf{x}'\|^2}{2\ell^2}\right)$$

where $\ell$ is the length-scale and $\sigma_f^2$ is the signal variance.

Given $n$ observations $\{(\mathbf{x}_i, y_i)\}$, the GP posterior at a new point $\mathbf{x}_*$ is Gaussian:

$$p(f_* | \mathbf{X}, \mathbf{y}, \mathbf{x}_*) = \mathcal{N}(\mu_*, \sigma_*^2)$$

$$\mu_* = \mathbf{k}_*^T (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{y}$$

$$\sigma_*^2 = k_{**} - \mathbf{k}_*^T (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{k}_*$$

where:
- $\mathbf{K}_{ij} = k(\mathbf{x}_i, \mathbf{x}_j)$ is the $n \times n$ kernel matrix
- $\mathbf{k}_{*,i} = k(\mathbf{x}_*, \mathbf{x}_i)$ is the cross-kernel vector
- $\sigma_n^2$ is the observation noise variance

### Expected Improvement Acquisition Function

For minimization, the Expected Improvement (EI) at $\mathbf{x}$ given best observed value $f^* = \min_i y_i$ is:

$$\text{EI}(\mathbf{x}) = \mathbb{E}[\max(f^* - f(\mathbf{x}), 0)]$$

For a GP surrogate this evaluates analytically:

$$\text{EI}(\mathbf{x}) = (f^* - \mu_*)\,\Phi(z) + \sigma_*\,\phi(z), \quad z = \frac{f^* - \mu_*}{\sigma_*}$$

where $\Phi$ is the standard normal CDF and $\phi$ is the PDF.

### Fixed-Size GP Implementation

To avoid dynamic memory allocation, the kernel matrix is pre-allocated at compile-time with
maximum capacity $M$. When only $n < M$ observations are present, the
inactive rows and columns are padded with an identity block, preserving positive-definiteness
and allowing Gaussian elimination with the full $M \times M$ system. The resulting alpha vector
$\boldsymbol{\alpha} = (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{y}$ has zeros in the
inactive entries.

## Complexity Analysis

| Case    | Time per iteration       | Space           | Notes |
|---------|--------------------------|-----------------|-------|
| Best    | $O(M^3)$                 | $O(M^2 + NP)$   | Dominated by GP solve |
| Average | $O(M^3 + C \cdot P)$     | $O(M^2 + NP)$   | $C$ candidates, $P$ parameters |
| Worst   | $O(M^3 + C \cdot P)$     | $O(M^2 + NP)$   | Same as average |

Where $M$ = maximum observations, $C$ = candidate count, $P$ = parameter count, $N$ = current number of observations.

All memory is pre-allocated at compile-time on the stack — no dynamic allocation required.

## Step-by-Step Walkthrough

**Example**: minimize $f(x) = (x - 0.5)^2$ on $x \in [0, 1]$.

1. **Initialization**: draw 2 random points, e.g. $x_1 = 0.12$, $x_2 = 0.87$.
2. **Evaluate**: $y_1 = 0.1444$, $y_2 = 0.1369$. Best so far $f^* = 0.1369$.
3. **Build GP**: compute $2 \times 2$ kernel matrix $\mathbf{K}$ with hyperparameters
   $\ell=1$, $\sigma_f=1$, $\sigma_n=0.01$.
4. **Solve**: $\boldsymbol{\alpha} = (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{y}$.
5. **Maximize EI**: sample 100 candidates uniformly, compute $\text{EI}(x_k)$ for each,
   select $x_3 = \arg\max \text{EI}$.
6. **Evaluate**: $y_3 = f(x_3)$. Update GP. Repeat.
7. **Convergence** (after 20 iterations): best point converges near $x = 0.5$.

## Pitfalls & Edge Cases

- **GP kernel matrix conditioning**: If two observations are nearly identical, $\mathbf{K} + \sigma_n^2 \mathbf{I}$
  can be ill-conditioned. The noise variance $\sigma_n^2$ acts as a regularizer; set it $\geq 10^{-3}$ for stability.
- **Exploration vs exploitation**: A small candidate sample count (below ~30) may miss the EI maximum in high dimensions.
  Increase the candidate count for $P > 3$ parameters.
- **Bounds scaling**: EI maximization by uniform sampling from $[-1, 1]^P$ assumes a normalized search space.
  Provide appropriate bounds and the optimizer will rescale internally.
- **Fixed capacity**: The maximum observation budget is a compile-time parameter. Set it large enough for the intended
  number of BO iterations. Exceeding the capacity at runtime triggers a precondition failure.

## Variants & Generalizations

- **Upper Confidence Bound (UCB)**: alternative to EI: $\text{UCB}(\mathbf{x}) = \mu_* - \kappa \sigma_*$.
  More explicit trade-off via $\kappa$.
- **GP hyperparameter optimization**: Maximize the log marginal likelihood over $\ell$, $\sigma_f$, $\sigma_n$
  to adapt the surrogate to the observed data automatically.
- **Gradient-enhanced BO**: When gradients are available, use GP regression on $(f, \nabla f)$ jointly
  for faster convergence.
- **Batch BO**: Select $q > 1$ candidates per iteration (parallel evaluations) using the $q$-EI criterion.

## Applications

- Controller weight tuning (e.g., MPC state/control cost weights Q, R)
- Hyperparameter optimisation for machine learning models
- Experiment design in system identification
- Any engineering optimisation where each evaluation is costly (simulation, hardware in the loop)

## Connections to Other Algorithms

- **Gaussian Process (GP)**: The surrogate model is a GP; the Kalman filter is formally equivalent to
  GP regression with a Markovian kernel.
- **Gaussian Elimination / Cholesky**: Used to solve the GP kernel system $(\mathbf{K} + \sigma_n^2 \mathbf{I})\boldsymbol{\alpha} = \mathbf{y}$.
- **Gradient Descent**: BO is the gradient-free counterpart — preferred when the objective is non-differentiable
  or noisy.
- **Expectation-Maximisation (EM)**: Both maintain a probabilistic model refined by data; EM operates on
  latent-variable likelihood while BO operates on a surrogate of an arbitrary black-box.

## Numerical Properties

- **Real-time suitability**: No — the $O(M^3)$ GP linear solve at each iteration precludes hard real-time use.
  Suitable for offline calibration or non-real-time configuration stages.
- **Precision**: Floating-point arithmetic only. Gaussian process kernels require transcendental
  functions — exponential, complementary error function, square root — that are incompatible with
  fixed-point representations. The kernel matrix inversion loses precision for very flat kernels
  ($\ell \gg$ domain width). Length-scale should remain in the range $[0.1, 10]$ relative to the
  normalised parameter domain.
- **Noise regularisation**: The diagonal noise term $\sigma_n^2 \mathbf{I}$ must be positive to keep the
  kernel matrix positive-definite; values below $10^{-3}$ risk ill-conditioning.
- **Memory**: All storage is pre-allocated at compile-time — the maximum observation budget and the
  candidate count are both compile-time parameters. No dynamic allocation is required.

## References & Further Reading

- Brochu, Cora, de Freitas, "A Tutorial on Bayesian Optimization of Expensive Cost Functions", 2010
- Rasmussen & Williams, *Gaussian Processes for Machine Learning*, MIT Press, 2006 — Chapter 2 (GP regression), Chapter 5 (model selection)
- Snoek, Larochelle, Adams, "Practical Bayesian Optimization of Machine Learning Algorithms", NeurIPS 2012

