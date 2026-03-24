# Infinite Impulse Response (IIR) Filter

## Overview & Motivation

An Infinite Impulse Response (IIR) filter uses feedback from its own past outputs to achieve a desired frequency response. IIR filters can match the selectivity of high-order FIR filters using far fewer coefficients, making them efficient for resource-constrained embedded systems. The trade-off is potential instability and nonlinear phase.

## Mathematical Theory

### Core Definition

A causal IIR filter with $P$ feedforward coefficients and $Q$ feedback coefficients computes:

$$y[n] = \sum_{i=0}^{P-1} b_i \cdot x[n-i] + \sum_{i=0}^{Q-1} a_i \cdot y[n-i]$$

where:
- $b_i$ are the feedforward (numerator) coefficients
- $a_i$ are the feedback (denominator) coefficients
- $x[n-i]$ are past input samples
- $y[n-i]$ are past output samples

**Sign convention**: The implementation uses addition for the feedback term. This means the feedback coefficients $a_i$ must be provided with the appropriate sign (negated relative to the standard form $y[n] = \sum b_i x[n-i] - \sum a_i y[n-i]$).

### Transfer Function

$$H(z) = \frac{\sum_{i=0}^{P-1} b_i z^{-i}}{1 - \sum_{i=0}^{Q-1} a_i z^{-i}}$$

### Stability Criterion

The filter is stable if and only if all poles of $H(z)$ lie strictly inside the unit circle ($|z| < 1$). This must be verified at design time.

## Complexity Analysis

| Case    | Time       | Space       | Notes                                  |
|---------|------------|-------------|----------------------------------------|
| Best    | $O(P+Q)$  | $O(P+Q)$   | Disabled (passthrough)                 |
| Average | $O(P+Q)$  | $O(P+Q)$   | Feedforward + feedback sums            |
| Worst   | $O(P+Q)$  | $O(P+Q)$   | Same — sequential, no branching        |

Time is per sample. Space stores coefficient and delay line buffers.

## Step-by-Step Walkthrough

Given a first-order IIR (1 feedforward, 1 feedback) with $b_0 = 0.5$, $a_0 = 0.3$ and input $x = [1.0, 0.0, 0.0]$:

| $n$ | $x[n]$ | $y[n-1]$ | $y[n] = 0.5 \cdot x[n] + 0.3 \cdot y[n-1]$ |
|-----|---------|-----------|----------------------------------------------|
| 0   | 1.0     | 0.0       | 0.5                                          |
| 1   | 0.0     | 0.5       | 0.15                                         |
| 2   | 0.0     | 0.15      | 0.045                                        |

The output decays exponentially — characteristic of IIR filters with $|a_0| < 1$.

## Pitfalls & Edge Cases

- **Instability**: If any pole has magnitude $\geq 1$, the output grows without bound. Always verify pole locations before deployment.
- **Coefficient quantization**: Fixed-point representation of feedback coefficients can shift pole locations, potentially pushing them outside the unit circle.
- **Limit cycles**: In fixed-point implementations, small-signal oscillations can occur due to quantization of the feedback path.
- **Overflow in feedback**: The recursive nature means accumulated rounding errors compound over time. Consider using higher-precision accumulators (Q31 for Q15 data).
- **Initial conditions**: Nonzero initial conditions in the output buffer affect the transient response.

## Variants & Generalizations

- **Direct Form I**: Separate feedforward and feedback computation (as implemented).
- **Direct Form II**: Reduces memory by sharing a single delay line, but is more sensitive to quantization.
- **Cascade (Second-Order Sections)**: Factor the transfer function into biquad sections for improved numerical stability.
- **Lattice structure**: Alternative topology with better quantization properties.

## Applications

- Butterworth, Chebyshev, and elliptic filter implementations
- DC removal (high-pass with pole near $z = 1$)
- Resonant filtering and equalization
- Control loop compensation filters

## Connections to Other Algorithms

- **FIR Filter**: FIR is a special case of IIR with $Q = 0$ (no feedback). FIR is always stable but requires more coefficients for comparable selectivity.
- **Frequency Response**: The `FrequencyResponse` class evaluates $H(e^{j\omega})$ for both FIR and IIR filters by accepting both $b$ and $a$ coefficient arrays.
- **Kalman Filter**: Kalman filters can be viewed as optimal adaptive IIR filters with time-varying coefficients.
