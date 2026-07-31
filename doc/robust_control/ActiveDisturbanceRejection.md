# Active Disturbance Rejection Control (ADRC + ESO)

## Overview & Motivation

Active Disturbance Rejection Control addresses a fundamental tension in feedback design: high-performance control normally requires an accurate plant model, yet accurate models are expensive to identify and degrade with temperature, load, and wear. ADRC resolves this by treating everything beyond a known input gain — unmodeled dynamics, parameter variation, and external disturbances — as a single lumped signal called the *total disturbance*. An Extended State Observer (ESO) estimates this signal in real time, and the control law subtracts the estimate before issuing the command. What remains behaves like a clean chain of integrators that a simple PD law can regulate with textbook bandwidth.

The practical payoff on embedded hardware is significant: you need only one plant number ($b_0$, the rough input gain) and two tuning dials. The controller then survives a bad model because any mismatch is absorbed into the disturbance estimate.

## Mathematical Theory

### Plant Representation

An $n$-th order SISO plant is written as the canonical integrator chain plus a total-disturbance term $f$:

$$y^{(n)} = f(t, y, \dot{y}, \ldots, d) + b_0 u$$

where $f$ captures unmodeled dynamics, nonlinearities, and external loads; $b_0$ is a nominal input-gain estimate; and $u$ is the control input.

### Extended State Observer

Augmenting the $n$ plant states with $x_{n+1} = f$ yields an $(n+1)$-dimensional system. The continuous ESO is a Luenberger-type observer driven by the output error:

$$\dot{\hat{x}}_i = \hat{x}_{i+1} + \beta_i (y - \hat{x}_1), \quad i = 1, \ldots, n$$
$$\dot{\hat{x}}_{n+1} = \beta_{n+1} (y - \hat{x}_1)$$

with the convention $\hat{x}_{n+1} = \hat{f}$ and $\hat{x}_2$ through $\hat{x}_n$ as derivative estimates.

The forward-Euler discretization used here is:

$$\hat{x}_i[k+1] = \hat{x}_i[k] + T_s \bigl(\beta_i \, e[k] + \hat{x}_{i+1}[k]\bigr), \quad e[k] = y[k] - \hat{x}_1[k]$$

with $b_0 u[k-1]$ injected into the $(n)$-th state to drive the highest derivative.

### Bandwidth Parameterization (Gao)

All observer poles are placed at $-\omega_o$ (Gao's bandwidth parameterization). The resulting gains follow the binomial expansion of $(\lambda + \omega_o)^{n+1}$:

$$\beta_i = \binom{n+1}{i} \omega_o^i, \quad i = 1, \ldots, n+1$$

All control poles are placed at $-\omega_c$ via the expansion of $(\lambda + \omega_c)^n$:

$$k_i = \binom{n}{i} \omega_c^i, \quad i = 1, \ldots, n$$

For a second-order plant ($n = 2$):

$$\beta = [3\omega_o,\; 3\omega_o^2,\; \omega_o^3], \quad k = [\omega_c^2,\; 2\omega_c]$$

### Control Law

After disturbance estimation the control is:

$$u = \frac{u_0 - \hat{f}}{b_0}, \qquad u_0 = k_1(r - \hat{x}_1) - \sum_{i=2}^{n} k_i \hat{x}_i$$

Substituting into the plant equation and using $\hat{f} \approx f$ gives the closed-loop residual $y^{(n)} \approx u_0$, a pure integrator chain under a PD law — independent of the original plant dynamics.

## Complexity Analysis

| Case    | Time   | Space  | Notes                                   |
|---------|--------|--------|-----------------------------------------|
| Best    | $O(n)$ | $O(n)$ | Linear sweep over $n+1$ ESO states      |
| Average | $O(n)$ | $O(n)$ | Same; gains precomputed at construction |
| Worst   | $O(n)$ | $O(n)$ | No branching in the hot path            |

Gains are computed once at construction from closed-form binomial formulas in $O(n)$ time. The `Compute` hot path is a pair of $O(n)$ loops with no dynamic allocation.

## Step-by-Step Walkthrough

Second-order plant ($n=2$), $\omega_o = 30$, $\omega_c = 6$, $b_0 = 1$, $T_s = 0.001$ s.

Observer gains: $\beta_1 = 90$, $\beta_2 = 2700$, $\beta_3 = 27000$.  
Control gains: $k_p = 36$, $k_d = 12$.

At sample $k$ with state $\hat{x} = [\hat{y}, \hat{\dot{y}}, \hat{f}]$, measurement $y[k]$, reference $r$:

1. Output error: $e = y[k] - \hat{y}$.
2. Inject correction into all three states: $\hat{x}_i \mathrel{+}= T_s \beta_i e$.
3. Chain integration: $\hat{y} \mathrel{+}= T_s \hat{\dot{y}}$; then $\hat{\dot{y}} \mathrel{+}= T_s b_0 u[k-1]$.
4. PD law on integrator chain: $u_0 = k_p(r - \hat{y}) - k_d \hat{\dot{y}}$.
5. Disturbance cancellation: $u = (u_0 - \hat{f}) / b_0$.

After a transient of roughly $5/\omega_o \approx 0.17$ s the observer converges; the output tracks $r$ with bandwidth $\omega_c$.

## Pitfalls & Edge Cases

**ESO peaking.** Large initial estimation errors drive high-magnitude corrections, temporarily saturating the actuator. Mitigation: initialize the observer near the first measurement, or schedule $\omega_o$ upward from a low value during the first few samples.

**Observer bandwidth vs. noise.** Increasing $\omega_o$ speeds convergence but amplifies measurement noise because $\beta_3 = \omega_o^3$ grows cubically. A practical rule of thumb is $\omega_o \in [3\omega_c, 10\omega_c]$.

**$b_0$ mismatch.** The ESO is robust to moderate mismatch (factor of 2–3), but large errors shrink the stability margin. If $b_0 \gg b_\text{true}$ the effective loop gain drops and response slows; if $b_0 \ll b_\text{true}$ the loop gain rises and may oscillate.

**Euler discretization accuracy.** The forward-Euler ESO introduces phase lag proportional to $\omega_o T_s$. Keeping $\omega_o T_s \ll 1$ (e.g., $\omega_o T_s \leq 0.1$) maintains accuracy; at higher $\omega_o T_s$ a ZOH or bilinear discretization is preferred.

**Integer overflow in gain computation.** Binomial coefficients are computed with integer arithmetic at compile time. For large orders or very high bandwidths the intermediate product may exceed `std::size_t` before the division; keep $n \leq 5$ in practice.

## Variants & Generalizations

**Nonlinear ESO (NESO).** Replace the linear correction $\beta_i e$ with Han's fal function to reduce peaking while preserving fast convergence.

**Discrete ESO.** Exact discretization of the observer (ZOH or pole-matched) improves accuracy when $\omega_o T_s$ is not small.

**Higher-order plants.** The template parameter `Order` generalizes the same bandwidth-parameterized structure to $n > 2$ — gains grow binomially and the `Compute` loop extends automatically.

**Multi-input / multi-output (MIMO).** Each output channel runs an independent ADRC; cross-coupling is absorbed into the respective disturbance estimates.

## Applications

- Electric motor drives (rejects friction, load torque, and back-EMF variation with a single $b_0$ estimate).
- Attitude and position control of UAVs and satellites (absorbs aerodynamic and thruster uncertainty).
- Industrial process control where the plant model is poorly known or time-varying.
- Hard-disk drive servo (high-bandwidth disturbance rejection without a detailed head-media model).

## Connections to Other Algorithms

- **Luenberger Observer** — the ESO is a Luenberger observer augmented with one extra disturbance state.
- **Disturbance Observer (DOB)** — the transfer-function sibling; DOB works in the frequency domain while ESO works in the state-space domain.
- **PID** — ADRC generalizes PID: a first-order ADRC with proportional-plus-integral action recovers a PI with disturbance feed-forward.
- **LQR / LQI** — state-feedback alternatives that require a full model; ADRC trades optimality for model-independence.

## References & Further Reading

- J. Han, "From PID to Active Disturbance Rejection Control," *IEEE Transactions on Industrial Electronics*, vol. 56, no. 3, pp. 900–906, 2009.
- Z. Gao, "Scaling and Bandwidth-Parameterization Based Controller Tuning," *Proceedings of the American Control Conference*, 2003, pp. 4989–4996.
- R. Miklosovic, A. Radke, Z. Gao, "Discrete implementation and generalization of the extended state observer," *ACC*, 2006.
