# Step / Transient-Response Metrics

## Overview & Motivation

When a control system or filter receives a step input, its output traces a transient trajectory before settling at the final value. Quantifying that trajectory with standardised scalar metrics — rise time, settling time, percent overshoot, peak time, and steady-state error — is the primary acceptance test for any closed-loop design. These metrics translate the raw sample sequence into the language of control specifications, allowing automated pass/fail decisions without manual inspection of time-domain plots.

## Mathematical Theory

### Definitions

Let $y[k]$, $k = 0, \ldots, N-1$ be the sampled step response and $y_{ss}$ the steady-state value. The sample period is $\Delta t$.

**Rise Time** $T_r$

The elapsed time for the response to travel from 10 % to 90 % of steady state:

$$T_r = (k_{90} - k_{10})\,\Delta t$$

where $k_{10} = \min\{k : y[k] \ge 0.1\,y_{ss}\}$ and $k_{90} = \min\{k \ge k_{10} : y[k] \ge 0.9\,y_{ss}\}$.

**Settling Time** $T_s$

The first time after which the response remains permanently inside the band $[(1-\delta)y_{ss},\,(1+\delta)y_{ss}]$ (typically $\delta = 0.02$):

$$T_s = (k^* + 1)\,\Delta t, \quad k^* = \max\{k : |y[k] - y_{ss}| > \delta\,|y_{ss}|\}$$

**Percent Overshoot** $\%OS$

$$\%OS = 100\,\frac{y_{\max} - y_{ss}}{y_{ss}}, \quad y_{\max} = \max_k y[k]$$

For an underdamped second-order system with damping ratio $\zeta$:

$$\%OS = 100\,\exp\!\left(-\frac{\pi\zeta}{\sqrt{1-\zeta^2}}\right)$$

**Peak Time** $T_p$

$$T_p = k_p\,\Delta t, \quad k_p = \arg\max_k y[k]$$

For a continuous underdamped second-order system with natural frequency $\omega_n$:

$$T_p = \frac{\pi}{\omega_n\sqrt{1-\zeta^2}}$$

**Steady-State Error** $e_{ss}$

$$e_{ss} = r - \bar{y}_{\text{tail}}$$

where $r$ is the reference (command) value and $\bar{y}_{\text{tail}}$ is the mean of the final quarter of the response buffer, providing a robust estimate of the achieved steady state.

## Complexity Analysis

| Case    | Time     | Space  | Notes                                      |
|---------|----------|--------|--------------------------------------------|
| All     | $O(N)$   | $O(1)$ | Single forward pass; no auxiliary storage  |

Each metric requires at most one traversal of the $N$-element vector. The tail-mean for steady-state error adds a constant-fraction second scan of the same data — still $O(N)$ total.

## Step-by-Step Walkthrough

Consider a 10-sample ramp to $y_{ss} = 1$ followed by a constant plateau (N = 20):

```
k:  0  1  2  3  4  5  6  7  8  9 10 11 …
y: 0  .1 .2 .3 .4 .5 .6 .7 .8 .9 1  1  …
```

- **Rise Time:** $k_{10} = 1$ (first sample $\ge 0.1$), $k_{90} = 9$ (first sample $\ge 0.9$). $T_r = 8\,\Delta t$.
- **Settling Time:** With $\delta = 0.02$, last sample outside the band is $k = 9$. $T_s = 10\,\Delta t$.
- **Percent Overshoot:** $y_{\max} = 1.0 = y_{ss}$, so $\%OS = 0$.
- **Peak Time:** $k_p = 10$ (first occurrence of max). $T_p = 10\,\Delta t$.
- **Steady-State Error:** Tail mean $= 1.0$, reference $= 1.0$. $e_{ss} = 0$.

## Pitfalls & Edge Cases

**Zero steady state.** Division by $y_{ss}$ in percent overshoot is guarded; the function returns zero when $y_{ss} = 0$ to avoid a NaN.

**Non-monotone ramp.** If the response crosses 90 % before 10 % (e.g., DC offset or wrong initial condition), $k_{10}$ may be found after the first 90 % crossing. The implementation returns the first pair that satisfies the threshold order.

**Oscillatory settling.** Settling time is defined as the last time the trajectory leaves the band, not the first time it enters it. Repeated crossings near the boundary extend the metric correctly.

**Finite buffer.** With a bounded vector of length $N$, if the response has not yet settled by the final sample, `SettlingTime` returns $N\,\Delta t$ and `RiseTime` returns $(N-1)\,\Delta t$ as conservative bounds.

**Tail-mean length.** Using the last $\lfloor N/4 \rfloor + 1$ samples for the steady-state estimate assumes the transient has decayed to within numerical noise by that point. Poorly chosen $N$ relative to the system time constant degrades the estimate.

## Variants & Generalizations

- **Delay Time** $T_d$: the time to reach 50 % of steady state — obtainable with the same threshold-scan pattern.
- **Band-relative rise time**: using a band other than 10–90 % (e.g., 20–80 %) is a trivial parameter change.
- **Multi-channel:** applying the scalar functions element-wise to each row of a response matrix generalises to MIMO systems without algorithmic change.

## Applications

- Automated controller tuning acceptance: verify that a PID or LQR design meets specification ($T_r < T_{r,\text{spec}}$, $\%OS < \%OS_{\text{spec}}$, etc.).
- Filter characterisation: measure the transient of a step fed through an IIR or FIR filter.
- Hardware-in-the-loop test harnesses: compute metrics directly from sampled actuator responses.

## Connections to Other Algorithms

- **Statistics** (this library): the tail-mean for steady-state error replicates the `Mean` function on a sub-range.
- **LinearTimeInvariant**: the primary source of step responses whose metrics are evaluated here.
- **Filters/active** (Kalman, EKF): step-excitation tests use these metrics to validate estimator transient behaviour.
- **Controllers**: PID and LQR tuning loops iterate until all five metrics satisfy design targets.

## References & Further Reading

- K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008. Chapter 10.
- G. F. Franklin, J. D. Powell, and A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019. Chapter 3.
- N. S. Nise, *Control Systems Engineering*, 8th ed., Wiley, 2019. Chapter 4.
