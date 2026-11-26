# Continuous-Gain PID with Measured Step

## Overview & Motivation

A discrete PID usually takes per-sample gains: the integral gain already holds the sample period, and the derivative
gain already holds its inverse. That works when the loop runs at an exact, fixed rate. A loop driven by a real
clock or by sensor samples rarely does. Its period jitters, and it changes when the loop rate is changed. Per-sample
gains then lose their meaning, and the operator must retune them.

This controller takes the continuous-time gains instead: proportional, integral per second and derivative in
seconds. Each step also takes the measured time since the previous step. The integral and derivative terms scale by
that real interval, so the gains keep their physical meaning at any rate.

## Mathematical Theory

### Velocity form with a variable step

With error $e_k = r_k - y_k$, step length $h_k$ and derivative estimate $d_k = (e_k - e_{k-1})/h_k$, the output
changes by

$$\Delta u_k = K_p\,(e_k - e_{k-1}) + K_i\,h_k\,e_k + K_d\,(d_k - d_{k-1})$$

and $u_k = \operatorname{clamp}(u_{k-1} + \Delta u_k,\ u_{\min},\ u_{\max})$. Summing the increments without the clamp
gives the position form

$$u_k = K_p\,e_k + K_i \sum_j h_j\,e_j + K_d\,d_k.$$

The integral is the rectangle rule over the real step lengths. The derivative is the backward difference over the
real last step.

### Start-up

After a reset the error history is zero. The first step therefore applies the full proportional term
$K_p\,e_0$. The first step has no derivative estimate, so $d_0 = 0$: a set-point step does not kick the output.

### Anti-windup

The clamp acts on the accumulated output. While the output stays on a limit, each step adds at most one increment
past it. The integral part therefore cannot grow without bound, and the output leaves the limit on the first step
whose increment points back.

### Equivalence

At a fixed step $h$, from the second step on, the law equals the per-sample velocity PID with gains $K_p$,
$K_i h$ and $K_d / h$.

## Complexity Analysis

| Operation | Time   | Space  | Notes                                        |
|-----------|--------|--------|----------------------------------------------|
| Step      | $O(1)$ | $O(1)$ | One division, a few multiply-adds, a clamp   |
| State     | —      | $O(1)$ | Output, last error, last derivative, a count |

## Step-by-Step Walkthrough

Gains $K_p = 0$, $K_i = 1$, $K_d = 0$, a constant error of 1 and steps of 0.01, 0.03 and 0.06 s:

1. The output is $1 \times 0.01 \times 1 = 0.01$.
2. It becomes $0.01 + 0.03 = 0.04$.
3. It becomes $0.04 + 0.06 = 0.10$, the integral of the error over 0.1 s.

## Pitfalls & Edge Cases

- **Bad interval**: a step length that is zero, negative or not finite leaves the state unchanged. A stalled or
  restarted clock therefore cannot divide by zero.
- **Derivative noise**: the backward difference amplifies measurement noise as the step gets shorter. Filter the
  measurement, or keep $K_d$ small.
- **Gain changes**: new gains act from the next step. The velocity form keeps the integral part, so a gain change
  does not bump the output.
- **Limits**: the minimum must be below the maximum.

## Variants & Generalizations

- Set-point weighting on the proportional and derivative terms.
- A first-order filter on the derivative.
- Back-calculation from the applied value, as in the incremental PI with back-calculation.

## Applications

- Loops driven by sensor samples with a jittering period.
- Cascaded loops that run at different rates but share tuning values in physical units.

## Connections to Other Algorithms

- [PID Controller](Pid.md): the per-sample form at a fixed rate.
- [Incremental PI with Back-Calculation](PiIncremental.md): the same velocity form, with the applied value fed back.

## References & Further Reading

- K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006, chapter 13.
