# Moving Average (Running-Sum Boxcar)

## Overview & Motivation

Sensor streams in embedded systems are corrupted by zero-mean white noise. The simplest way to
attenuate that noise is to average consecutive samples. The moving average does this with a sliding
window of length N, treating every sample inside the window equally. Its critical property for
embedded use is that the recursive running-sum form requires exactly one addition, one subtraction,
and one multiply per output sample — constant work regardless of how large the window is.

## Mathematical Theory

### Core Definitions

Let $x[n]$ be the input sequence. The length-$N$ boxcar output is:

$$y[n] = \frac{1}{N} \sum_{k=0}^{N-1} x[n-k]$$

### Recursive Form

Expanding $y[n]$ in terms of $y[n-1]$:

$$y[n] = y[n-1] + \frac{x[n] - x[n-N]}{N}$$

Equivalently, maintain a running sum $S[n] = \sum_{k=0}^{N-1} x[n-k]$:

$$S[n] = S[n-1] + x[n] - x[n-N], \qquad y[n] = \frac{S[n]}{N}$$

### Frequency Response

The discrete-time transfer function is:

$$H(z) = \frac{1}{N} \sum_{k=0}^{N-1} z^{-k} = \frac{1}{N} \cdot \frac{1 - z^{-N}}{1 - z^{-1}}$$

The magnitude response is a $\mathrm{sinc}$-shaped envelope:

$$|H(e^{j\omega})| = \frac{1}{N} \left| \frac{\sin(N\omega/2)}{\sin(\omega/2)} \right|$$

DC gain is exactly 1. Group delay is constant at $(N-1)/2$ samples — the filter is linear phase.

## Complexity Analysis

| Case | Time   | Space  | Notes                          |
|------|--------|--------|--------------------------------|
| All  | $O(1)$ | $O(N)$ | One add, one sub, one multiply |

The $O(1)$ time cost is the key advantage over the naive $O(N)$ direct-form sum.

## Step-by-Step Walkthrough

Input: unit step $x[n] = 1$ for $n \ge 0$, $N = 4$, initial state all zeros.

| $n$ | $x[n]$ | $x[n-4]$ | $S[n]$ | $y[n]$ |
|-----|--------|----------|--------|--------|
| 0   | 1      | 0        | 1      | 0.25   |
| 1   | 1      | 0        | 2      | 0.50   |
| 2   | 1      | 0        | 3      | 0.75   |
| 3   | 1      | 0        | 4      | 1.00   |
| 4   | 1      | 1        | 4      | 1.00   |

The output ramps up linearly and then holds at 1 once the window is fully filled.

## Pitfalls & Edge Cases

- **Running-sum drift**: floating-point rounding accumulates in $S[n]$ over long constant runs.
  For very long sequences, periodically recompute the sum directly from the window contents.
- **Startup transient**: the first $N-1$ outputs are averages of fewer than $N$ samples unless the
  window is pre-filled with an initial value.
- **Window length 1**: reduces to pass-through — no smoothing occurs.
- **Poor frequency selectivity**: sidelobes in the stopband are only $-13\,\mathrm{dB}$ below the
  passband; use a Biquad cascade when sharp roll-off is needed.

## Variants & Generalizations

- **Weighted moving average** (triangular, Gaussian): tap weights are no longer uniform; loses the
  $O(1)$ recursive property.
- **Exponential moving average (EMA)**: an IIR approximation requiring only one multiply-add and no
  delay line; trades linear phase for lower memory cost.
- **Savitzky-Golay filter**: fits a polynomial to the window, preserving peak shapes while
  smoothing — higher computational cost.
- **CIC filter**: cascaded integrator-comb, a multi-stage generalization used for decimation.

## Applications

- ADC noise reduction on sensor streams (accelerometers, temperature, pressure).
- Decimation pre-filter before down-sampling.
- Smoothing control-loop feedback signals to suppress high-frequency switching artifacts.
- Baseline/trend estimation when a flat passband and linear phase are required.

## Connections to Other Algorithms

- The FIR filter with uniform coefficients $b_k = 1/N$ is mathematically identical; this
  implementation exploits the uniformity for $O(1)$ computation.
- `ExponentialMovingAverage` solves the same smoothing problem with $O(1)$ memory at the cost of
  non-linear phase and an infinite impulse response.
- `SavitzkyGolayFilter` generalizes the boxcar by fitting a polynomial, preserving higher-order
  signal features.

## References & Further Reading

- S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 15 —
  Moving Average Filters (recursive running-sum form). Available at dspguide.com.
- A. V. Oppenheim & R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., §7.5.
