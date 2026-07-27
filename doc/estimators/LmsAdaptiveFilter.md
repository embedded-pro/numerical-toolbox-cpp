# LMS / NLMS Adaptive Filter

## Overview & Motivation

Real-time systems frequently need filters whose characteristics are unknown in advance or change over time. A fixed-coefficient FIR filter cannot cancel acoustic echo from an unknown loudspeaker-to-microphone path, equalize a time-varying channel, or identify the impulse response of a vibrating structure it has never seen before. The Least Mean Squares (LMS) algorithm and its normalized variant (NLMS) address this by adjusting a FIR filter's coefficients online, one sample at a time, without requiring matrix inversions or covariance storage.

## Mathematical Theory

### Prerequisites

Let $\mathbf{w} \in \mathbb{R}^{N}$ be the weight vector (FIR coefficients) and $\mathbf{x}_n \in \mathbb{R}^{N}$ the tapped-delay-line vector at time $n$, with $x_n$ as the newest sample and $x_{n-N+1}$ the oldest. The filter output is

$$y_n = \mathbf{w}_n^\top \mathbf{x}_n.$$

Given a desired signal $d_n$, the estimation error is

$$e_n = d_n - y_n.$$

### Core Definitions

The instantaneous squared error $J_n = e_n^2$ is a quadratic function of $\mathbf{w}_n$. Its gradient with respect to $\mathbf{w}$ is

$$\nabla J_n = -2 e_n \mathbf{x}_n.$$

### Derivation

LMS takes a noisy steepest-descent step using the instantaneous gradient:

$$\mathbf{w}_{n+1} = \mathbf{w}_n - \frac{\mu}{2} \nabla J_n = \mathbf{w}_n + \mu e_n \mathbf{x}_n.$$

NLMS divides the step by the input energy to make the effective step size input-level independent:

$$\mathbf{w}_{n+1} = \mathbf{w}_n + \frac{\mu}{\epsilon + \|\mathbf{x}_n\|^2} e_n \mathbf{x}_n,$$

where $\epsilon > 0$ is a small regularizer that prevents division by zero during silence ($\|\mathbf{x}_n\|^2 \approx 0$).

### Proof of Correctness

Under the independence assumption (successive input vectors are uncorrelated), the expected weight vector converges to the Wiener solution $\mathbf{w}^* = \mathbf{R}^{-1}\mathbf{p}$ (where $\mathbf{R}$ is the input autocorrelation matrix and $\mathbf{p}$ is the cross-correlation vector) provided

$$0 < \mu < \frac{2}{\lambda_{\max}(\mathbf{R})},$$

a sufficient condition being $\mu < 2 / (N \cdot \sigma_x^2)$. For NLMS the stability range simplifies to $0 < \mu < 2$.

## Complexity Analysis

| Case | Time   | Space  | Notes                                       |
|------|--------|--------|---------------------------------------------|
| Any  | $O(N)$ | $O(N)$ | Two dot products + one AXPY, all length $N$ |

No covariance matrix is stored; memory is exactly two vectors of length $N$ (weights and delay line).

## Step-by-Step Walkthrough

Consider a 2-tap system with true plant $\mathbf{w}^* = [0.5,\ 0.5]^\top$, $\mu = 0.1$, and a white input sequence.

| Step | Input | Delay line $\mathbf{x}$ | Output $y$ | Desired $d$ | Error $e$ | Weight update direction     |
|------|-------|-------------------------|------------|-------------|-----------|-----------------------------|
| 1    | 1.0   | [1.0, 0.0]              | 0.0        | 0.5         | 0.5       | $+0.05\mathbf{x}$           |
| 2    | 0.8   | [0.8, 1.0]              | 0.04       | 0.9         | 0.86      | $+0.086\mathbf{x}$          |
| …    | …     | …                       | …          | …           | …         | converges to $\mathbf{w}^*$ |

After many iterations the error approaches zero and the weights stabilize near $[0.5,\ 0.5]^\top$.

## Pitfalls & Edge Cases

- **Stability bound:** exceeding $\mu_{\max}$ causes exponential weight growth. For stationary white input, $\mu_{\max} = 2 / (N\sigma_x^2)$.
- **Misadjustment:** even at steady state, random gradient noise keeps the weights oscillating around $\mathbf{w}^*$, adding excess mean-squared error proportional to $\mu N$.
- **Slow convergence vs stability tradeoff:** small $\mu$ gives low misadjustment but slow tracking; large $\mu$ tracks fast but overshoots.
- **Silent input:** $\|\mathbf{x}_n\|^2 = 0$ causes a singularity in plain NLMS; $\epsilon$ prevents this but freezes adaptation during silence.
- **Non-persistent excitation:** if the input does not excite all directions, some weights may drift. Leaky-LMS ($\mathbf{w} \leftarrow (1-\rho)\mathbf{w} + \mu e \mathbf{x}$) bounds the drift.

## Variants & Generalizations

- **NLMS:** normalizes step by input energy; removes input-power dependence; recommended for signals with variable amplitude.
- **Leaky LMS:** adds a forgetting term $\rho \ll 1$ to bound weight norm under insufficient excitation.
- **Block LMS:** accumulates gradients over a block before updating; lower update rate but more FFT-friendly for long filters.
- **Sign-LMS / Sign-Error LMS:** replaces $e$ or $\mathbf{x}$ with their signs; extremely simple hardware implementation at the cost of convergence rate.
- **RLS:** replaces the stochastic gradient with the exact least-squares update; converges in $O(N)$ steps but requires $O(N^2)$ memory and time per update.

## Applications

- **Acoustic echo cancellation:** adapt the loudspeaker-to-microphone path in teleconferencing.
- **Adaptive noise cancellation:** estimate and subtract a correlated noise source (e.g. engine noise on an aircraft headset).
- **Channel equalization:** track a time-varying ISI channel in wireless modems.
- **Active noise control / vibration control:** generate an anti-phase signal to cancel mechanical vibration.
- **Online system identification:** recover the impulse response of an unknown plant in real time.

## Connections to Other Algorithms

- **RecursiveLeastSquares:** the exact-gradient counterpart; converges in far fewer samples but needs $O(N^2)$ storage and time per step. Prefer RLS for short filter lengths or fast convergence; prefer LMS/NLMS for embedded, resource-constrained paths.
- **FIR (fixed-coefficient):** LMS turns a fixed FIR into a self-tuning one; the underlying convolution is identical.
- **ConvolutionCorrelation:** used to compute the cross-correlation $\mathbf{p}$ in the batch Wiener solution; LMS approximates this online.

## References & Further Reading

- B. Widrow, M. E. Hoff, "Adaptive switching circuits," *IRE WESCON Convention Record*, 1960.
- S. Haykin, *Adaptive Filter Theory*, 5th ed., Pearson, 2014 — chapters 5–8.
- S. Sayed, *Fundamentals of Adaptive Filtering*, Wiley, 2003.
- D. G. Manolakis, V. K. Ingle, S. M. Kogon, *Statistical and Adaptive Signal Processing*, Artech House, 2005.
