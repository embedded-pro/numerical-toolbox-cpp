# Exponential Moving Average

## Overview & Motivation

Sensor data in embedded systems is noisy. A full moving-average buffer costs O(N) RAM and O(N)
computation per sample. The exponential moving average (EMA), also called a one-pole recursive
low-pass filter or leaky integrator, achieves useful smoothing with a single state word and one
multiply-accumulate per sample. It is the minimal-cost filter that tracks a slowly varying DC
level while suppressing high-frequency noise.

## Mathematical Theory

### Core Definition

Given a smoothing factor $\alpha \in (0, 1]$ and a discrete input sequence $x[n]$, the EMA
output $y[n]$ obeys the first-order linear recurrence:

$$y[n] = \alpha \, x[n] + (1 - \alpha) \, y[n-1]$$

with $y[-1] = y_0$ (the initial state, often zero).

### Rearranged Update Form

Expanding the recurrence:

$$y[n] = y[n-1] + \alpha \bigl(x[n] - y[n-1]\bigr)$$

This form has one subtraction and one multiply-accumulate; it avoids computing $(1-\alpha)$
explicitly and keeps the error term $(x[n] - y[n-1])$ small, which is numerically preferable in
fixed-point arithmetic.

### Pole Location and Stability

The transfer function in the $z$-domain is:

$$H(z) = \frac{\alpha}{1 - (1-\alpha)\,z^{-1}}$$

The single pole is at $z = 1 - \alpha$. For $\alpha \in (0, 1]$ the pole lies in $[0, 1)$, which
is strictly inside the unit circle, so the filter is unconditionally stable.

### DC Gain

Setting $z = 1$ (DC, $\omega = 0$):

$$H(1) = \frac{\alpha}{1 - (1-\alpha)} = 1$$

Unity DC gain: a constant input converges exactly to that constant.

### Cutoff Frequency

Matching the EMA pole to a first-order RC low-pass filter with time constant $\tau = 1/(2\pi f_c)$
and sample period $T_s = 1/f_s$:

$$\alpha = \frac{T_s}{\tau + T_s} = \frac{T_s}{\frac{1}{2\pi f_c} + T_s}$$

### Impulse Response

The impulse response is $h[n] = \alpha (1-\alpha)^n \, u[n]$, which decays geometrically —
hence the name "exponential" moving average.

## Complexity Analysis

| Case    | Time | Space | Notes                                  |
|---------|------|-------|----------------------------------------|
| Best    | O(1) | O(1)  | Single MAC per sample, one state word  |
| Average | O(1) | O(1)  | Identical regardless of input          |
| Worst   | O(1) | O(1)  | No data-dependent branches on hot path |

## Step-by-Step Walkthrough

Suppose $\alpha = 0.5$, $y[-1] = 0$, and the input is the unit impulse $x = [1, 0, 0, 0, \ldots]$.

| $n$ | $x[n]$ | $x[n] - y[n-1]$      | $\alpha \cdot \Delta$ | $y[n]$   |
|-----|--------|----------------------|-----------------------|----------|
| 0   | 1      | $1 - 0 = 1$          | $0.5$                 | $0.5$    |
| 1   | 0      | $0 - 0.5 = -0.5$     | $-0.25$               | $0.25$   |
| 2   | 0      | $0 - 0.25 = -0.25$   | $-0.125$              | $0.125$  |
| 3   | 0      | $0 - 0.125 = -0.125$ | $-0.0625$             | $0.0625$ |

Each output is half the previous, confirming $h[n] = (0.5)^{n+1}$.

## Pitfalls & Edge Cases

- $\alpha \to 0$: The filter barely reacts; the time constant grows toward infinity. At exactly
  zero the state never updates (division by zero in the cutoff formula).
- $\alpha = 1$: Pure passthrough — no smoothing at all; the output equals the input exactly.
- Initial transient: If the initial state differs significantly from the true signal level, the
  output will have a startup transient lasting approximately $1/\alpha$ samples. Seeding the
  state with the first sample eliminates this.
- Floating-point accumulation: Long runs of constant input may show residual drift at the level
  of floating-point epsilon relative to the signal magnitude; this is negligible for float32.

## Variants & Generalizations

- **Double EMA (DEMA)**: Applies the EMA twice and corrects for lag: $2\,y_1[n] - y_2[n]$.
  Reduces lag at the cost of one extra state word and multiply.
- **Kaufman Adaptive Moving Average (KAMA)**: Adjusts $\alpha$ dynamically based on market
  efficiency ratio; the same one-pole structure with a time-varying coefficient.
- **Exponentially Weighted Moving Variance**: Tracks variance alongside the mean using a second
  recurrence on the squared error.
- **Higher-order cascades**: Cascading $k$ EMA stages produces a Butterworth-like response with
  $k$ poles and $-20k$ dB/decade roll-off.

## Applications

- Sensor de-noising (IMU, ADC, temperature) where buffer RAM is scarce.
- DC-offset tracking and removal.
- Smoothed feedback signals in PID control loops.
- Battery voltage estimation.
- Signal envelope detection.

## Connections to Other Algorithms

- **FIR moving average**: Linear-phase alternative; symmetric impulse response gives zero group
  delay distortion but requires O(N) RAM and O(N) MACs.
- **Biquad cascade (IIR)**: Higher-order roll-off at the cost of two state words per section and
  potential instability if coefficients are not quantised carefully.
- **Kalman filter (scalar)**: The steady-state scalar Kalman gain is equivalent to an EMA with
  $\alpha = K$ determined by the noise ratio; EMA is the optimal estimator for a random-walk
  signal in Gaussian noise.

## References & Further Reading

- S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 19
  (Recursive / single-pole filters). Available free at dspguide.com.
- A. V. Oppenheim & R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Ch. 5 (IIR filter
  design).
- R. G. Lyons, *Understanding Digital Signal Processing*, 3rd ed., Ch. 6 (recursive filters).
