# Lead-Lag Compensator

## Overview & Motivation

Classical feedback loops require a mechanism to reshape the open-loop frequency response without the full overhead of a state-space design. A one-pole/one-zero compensator achieves this with three tuning parameters and two state words, making it practical for any microcontroller control loop. When phase margin is insufficient, a lead configuration injects extra phase near the gain crossover frequency, raising stability margin and permitting a higher bandwidth. When steady-state error is the concern, a lag configuration boosts low-frequency gain to drive the error toward zero while leaving the crossover region essentially unchanged.

## Mathematical Theory

### Continuous-Time Transfer Function

The compensator is defined in the Laplace domain as:

$$C(s) = K \cdot \frac{s + z}{s + p}$$

where $K$ is the overall gain, $z$ is the zero frequency (rad/s), and $p$ is the pole frequency (rad/s).

The DC gain is $C(0) = K \cdot z / p$.

- **Lead network** ($z < p$): the zero sits below the pole, so phase rises at mid frequencies and then falls again, providing a phase bump near crossover.
- **Lag network** ($z > p$): the pole sits below the zero, so the compensator acts as a high-gain integrator approximation at low frequencies and rolls back to unity at high frequencies.

### Bilinear (Tustin) Discretization

The bilinear transform substitutes $s \leftarrow \frac{2}{T_s} \cdot \frac{1 - z^{-1}}{1 + z^{-1}}$, mapping the entire left half of the $s$-plane to the interior of the unit circle in the $z$-plane and preserving stability.

Define $c = 2/T_s$. The numerator and denominator polynomials in $z$ are:

$$n_0 = K(c + z), \quad n_1 = K(z - c)$$
$$d_0 = c + p, \quad d_1 = p - c$$

Normalizing by $d_0$:

$$b_0 = \frac{n_0}{d_0}, \quad b_1 = \frac{n_1}{d_0}, \quad a_1 = \frac{d_1}{d_0}$$

### Discrete-Time Recurrence (Direct Form I)

$$y[n] = b_0 \, x[n] + b_1 \, x[n-1] - a_1 \, y[n-1]$$

The sign convention places the feedback term with a minus sign on $a_1$, so positive $a_1$ in the formula corresponds to a pole at $+a_1$ inside the unit disk.

## Complexity Analysis

| Case    | Time   | Space  | Notes                                          |
|---------|--------|--------|------------------------------------------------|
| Best    | $O(1)$ | $O(1)$ | Three multiply-adds, two state updates         |
| Average | $O(1)$ | $O(1)$ | Fixed instruction count per sample             |
| Worst   | $O(1)$ | $O(1)$ | No branching; deterministic real-time behavior |

Design (Tustin coefficient computation) is $O(1)$ and occurs once in the constructor.

## Step-by-Step Walkthrough

Parameters: $K=1$, $z=1\,\text{rad/s}$, $p=10\,\text{rad/s}$, $T_s = 0.01\,\text{s}$ (lead network).

1. $c = 2/0.01 = 200$
2. $n_0 = 1 \cdot (200 + 1) = 201$, $\quad n_1 = 1 \cdot (1 - 200) = -199$
3. $d_0 = 200 + 10 = 210$, $\quad d_1 = 10 - 200 = -190$
4. $b_0 = 201/210 \approx 0.9571$, $\quad b_1 = -199/210 \approx -0.9476$, $\quad a_1 = -190/210 \approx -0.9048$

Unit-step response (first two samples):

| $n$ | $x[n]$ | $b_0 x[n]$ | $b_1 x[n-1]$ | $-a_1 y[n-1]$ | $y[n]$ |
|-----|--------|------------|--------------|---------------|--------|
| 0   | 1      | 0.9571     | 0            | 0             | 0.9571 |
| 1   | 1      | 0.9571     | −0.9476      | 0.8664        | 0.8759 |

The first output (≈ 0.957) already exceeds the DC steady-state gain of 0.1, illustrating the phase-lead kick.

Verification of DC gain: $b_0 + b_1 = 2/210$; $1 + a_1 = 20/210$; ratio $= 2/20 = 0.1 = K \cdot z/p$.

## Pitfalls & Edge Cases

- **Near-Nyquist poles/zeros**: when $z$ or $p$ is comparable to $\pi/T_s$, the bilinear transform introduces frequency warping. Pre-warp the analog corner frequencies to $\hat\omega = (2/T_s)\tan(\omega T_s/2)$ before applying Tustin if exact placement matters.
- **Degenerate case $z = p$**: the compensator collapses to a pure gain $K$ with no dynamics. The discrete recurrence remains valid; the pole and zero cancel.
- **Unstable discretization**: a plant with a very fast analog pole relative to $T_s$ can map outside the unit disk; verify $|a_1| < 1$ after computing coefficients.
- **Floating-point accumulation**: the two state variables accumulate rounding error indefinitely. For long-running loops, periodic resets or double-precision state registers mitigate drift.

## Variants & Generalizations

- **Lead-lag cascade**: a lead section followed by a lag section in series provides simultaneous bandwidth improvement and steady-state accuracy. Reuse two first-order sections rather than chaining first-order blocks through a single instance.
- **Phase-lead only / phase-lag only**: selecting $z$ and $p$ exclusively achieves single-objective shaping; the structure is unchanged.
- **Pre-warped Tustin**: replace $z, p$ with $\hat z = (2/T_s)\tan(z T_s/2)$ and $\hat p = (2/T_s)\tan(p T_s/2)$ before computing Tustin coefficients to achieve exact frequency matching.
- **Second-order extension**: cascading two first-order sections or using a biquad second-order section enables lead-lag-lead or other compound shapes.

## Applications

- **Motor velocity loops**: a lead compensator raises phase margin to allow a higher proportional gain, which directly increases bandwidth and disturbance rejection.
- **Voltage regulators**: a lag compensator adds integrating action at mains frequency to eliminate steady-state regulation error without destabilizing the switching loop.
- **Flight control inner loops**: classical lead-lag design from Bode plots remains the dominant method for aircraft inner-loop stabilization due to its transparency and robustness to model uncertainty.
- **Thermal control**: a lag network boosts gain at low frequencies to null steady-state temperature offset while keeping the loop stable against slow sensor dynamics.

## Connections to Other Algorithms

- **BiquadCascade**: a second-order section generalizes to two poles and two zeros; cascading biquads is the standard approach when more than one lead-lag stage is required.
- **PidIncremental**: a PID controller contains an implicit lead-lag structure; tuning the derivative and integral terms is equivalent to placing the compensator zero and pole.
- **FrequencyResponse**: use the Bode magnitude and phase plots to verify that the shaped open-loop response achieves the desired gain crossover frequency and phase margin after adding the compensator.

## References & Further Reading

- G. F. Franklin, J. D. Powell, A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed. (2019) — Chapter 6: The Frequency-Response Design Method.
- K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers* (2008), Chapter 9: Frequency Domain Design.
- R. C. Dorf, R. H. Bishop, *Modern Control Systems*, 13th ed. (2017) — Lead and lag compensator design.
