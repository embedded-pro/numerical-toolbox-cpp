# Incremental PI with Back-Calculation

## Overview & Motivation

A PI regulator whose output passes through a limit it cannot see winds up: the integral keeps growing while the actuator is saturated, and the loop overshoots when the error finally reverses.
The [PID Controller](Pid.md) avoids this with a per-output clamp, which works when each output has its own independent limit.
Some limits are shared between several regulators instead.
A field-oriented current loop is the usual example: the $d$ and $q$ voltages are limited together to a circle, so neither axis alone knows when it is saturated.

`controllers::PiIncremental<T>` splits a sample into two calls.
`Propose` computes the unlimited demand from the error.
The caller applies whatever limit it needs, possibly to several regulators at once, and hands the value it actually applied back through `CommitRealized`.
The next increment then starts from what the actuator really did, so the integrator never accumulates against the limit.
It keeps four numbers, has no derivative term, no set-point state and no clamp, and costs two multiply-adds per sample.

## Mathematical Theory

### Velocity form

With proportional gain $K_p$ and per-sample integral gain $K_i$ (the continuous integral gain times the sample period), the incremental PI is

$$u[k] = u[k-1] + (K_p + K_i)\,e[k] - K_p\,e[k-1], \qquad e[k] = r[k] - y[k].$$

Summing the increments recovers the position form $u[k] = K_p\,e[k] + K_i \sum_{j \le k} e[j]$ when nothing is limited.

### Back-calculation

`Propose` evaluates the right-hand side and stores it as $u[k]$.
`CommitRealized(v)` replaces it with the applied value $v[k]$, so the next sample computes

$$u[k+1] = v[k] + (K_p + K_i)\,e[k+1] - K_p\,e[k].$$

While the actuator is saturated, $v[k]$ stays on the limit and each new proposal is that limit plus a single increment.
The integral state is therefore bounded by the limit plus one step, however long saturation lasts.
When the demand is inside the limit, $v[k] = u[k]$ and the law is the plain velocity form.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                              |
|----------------|--------|--------|------------------------------------|
| Propose        | $O(1)$ | $O(1)$ | One subtraction, two multiply-adds |
| CommitRealized | $O(1)$ | $O(1)$ | One store                          |
| SetTunings     | $O(1)$ | $O(1)$ | Precomputes $K_p + K_i$ and $-K_p$ |

The state is the two precomputed gains, the last output and the last error.

## Step-by-Step Walkthrough

$K_p = 2$, $K_i = 0.5$, reference $r = 5$, the actuator saturates at $6$:

| $k$ | $y[k]$ | $e[k]$ | Proposal $u[k]$                   | Applied $v[k]$ |
|-----|--------|--------|-----------------------------------|----------------|
| 0   | 0      | 5      | $0 + 2.5 \cdot 5 - 0 = 12.5$      | 6              |
| 1   | 1      | 4      | $6 + 2.5 \cdot 4 - 2 \cdot 5 = 6$ | 6              |
| 2   | 3      | 2      | $6 + 2.5 \cdot 2 - 2 \cdot 4 = 3$ | 3              |

Without the commit, step 1 would have started from 12.5 and proposed 12.5, and the excess would have kept growing for as long as the limit held.

## Pitfalls & Edge Cases

- **Call order.** Exactly one `CommitRealized` per `Propose`, after the limit is applied. Skipping it reverts to the plain velocity form without anti-windup.
- **Per-sample integral gain.** $K_i$ is per sample. Changing the sample rate requires recomputing it.
- **Retuning.** `SetTunings` keeps the accumulated output and the last error, so gain scheduling is bumpless; call `Reset` for a cold start.
- **Floating point only.** The class is restricted to floating-point types; fixed-point users should use the [PID Controller](Pid.md) with its Q15/Q31 instantiations.

## Variants & Generalizations

- Adding a derivative term gives the [PID Controller](Pid.md)'s recursion, at the cost of a second stored error.
- The same propose/commit split works for any incremental law whose state is its last output, for example an incremental lead-lag.

## Applications

- $d$/$q$ current regulators in field-oriented motor control, limited together to the modulation circle.
- Several regulators sharing one supply or one total-power budget.
- Any loop whose actuator limit depends on a measured quantity (bus voltage, temperature) that the regulator does not see.

## Connections to Other Algorithms

- [PID Controller](Pid.md): the same recursion with a derivative term, a stored set-point and a per-output clamp.
- [Saturation and Rate Limiter](SaturationRateLimiter.md): typical limits to apply between `Propose` and `CommitRealized`.

## References & Further Reading

- K. J. Åström and T. Hägglund, *Advanced PID Control*, ISA, 2006, Chapter 3 (integrator windup and back-calculation).
- K. J. Åström and B. Wittenmark, *Computer-Controlled Systems*, 3rd ed., Prentice Hall, 1997, Section 8.5 (velocity-form algorithms).
