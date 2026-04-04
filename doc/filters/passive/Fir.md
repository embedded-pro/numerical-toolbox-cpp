# Finite Impulse Response (FIR) Filter

## Overview & Motivation

A Finite Impulse Response (FIR) filter is a discrete-time filter whose impulse response has a finite duration. FIR filters are inherently stable, have linear phase properties (when symmetric), and are straightforward to design and implement. They are the default choice for applications requiring guaranteed stability and predictable phase behavior.

## Mathematical Theory

### Core Definition

A causal FIR filter of order $N-1$ (with $N$ taps) computes the output as a weighted sum of the current and past inputs (discrete convolution):

$$y[n] = \sum_{i=0}^{N-1} b_i \cdot x[n-i]$$

where:
- $b_i$ are the filter coefficients (tap weights)
- $x[n-i]$ are the current and $N-1$ most recent input samples
- $N$ is the number of taps (filter length)

### Transfer Function

In the z-domain, the FIR filter transfer function is a polynomial in $z^{-1}$:

$$H(z) = \sum_{i=0}^{N-1} b_i z^{-i}$$

Since the denominator is unity, FIR filters have all poles at the origin and are always stable.

### Frequency Response

Evaluating on the unit circle ($z = e^{j\omega}$):

$$H(e^{j\omega}) = \sum_{i=0}^{N-1} b_i e^{-j\omega i}$$

## Complexity Analysis

| Case    | Time   | Space  | Notes                              |
|---------|--------|--------|------------------------------------|
| Best    | $O(N)$ | $O(N)$ | Disabled (passthrough)             |
| Average | $O(N)$ | $O(N)$ | Convolution sum                    |
| Worst   | $O(N)$ | $O(N)$ | Same — no data-dependent branching |

Time is per sample. Space is for the coefficient and delay line buffers.

## Step-by-Step Walkthrough

Given a 3-tap FIR filter with coefficients $b = [0.25, 0.5, 0.25]$ and input sequence $x = [1.0, 2.0, 3.0, 4.0]$:

| $n$ | $x[n]$ | $x[n-1]$ | $x[n-2]$ | $y[n] = 0.25 x[n] + 0.5 x[n-1] + 0.25 x[n-2]$ |
|-----|--------|----------|----------|-----------------------------------------------|
| 0   | 1.0    | 0.0      | 0.0      | 0.25                                          |
| 1   | 2.0    | 1.0      | 0.0      | 1.0                                           |
| 2   | 3.0    | 2.0      | 1.0      | 2.0                                           |
| 3   | 4.0    | 3.0      | 2.0      | 3.0                                           |

## Pitfalls & Edge Cases

- **Group delay**: A linear-phase FIR filter introduces a delay of $(N-1)/2$ samples. This must be accounted for in real-time control loops.
- **Fixed-point overflow**: When using Q15 or Q31 arithmetic, ensure the sum of absolute coefficient values does not exceed 1.0 to prevent saturation.
- **Coefficient quantization**: Converting floating-point coefficients to fixed-point can distort the frequency response, especially for narrowband filters with large $N$.
- **Initial transient**: The first $N-1$ outputs are computed with zero-padded history; these may not be representative of steady-state behavior.

## Variants & Generalizations

- **Windowed design**: FIR coefficients are often designed by applying a window function (Hamming, Hann, Blackman) to the ideal impulse response.
- **Equiripple (Parks-McClellan)**: Minimax optimal design that minimizes the maximum deviation from the desired response.
- **Polyphase decomposition**: For efficient multirate (decimation/interpolation) implementations.

## Applications

- Low-pass, high-pass, band-pass, and band-stop filtering
- Anti-aliasing before downsampling
- Matched filtering in communications
- Moving average smoothing (special case with equal coefficients)

## Connections to Other Algorithms

- **IIR Filter**: IIR filters achieve sharper cutoffs with fewer coefficients but sacrifice guaranteed stability and linear phase.
- **FFT**: For large $N$, the convolution can be computed more efficiently via FFT-based overlap-add/save methods.
- **Window Functions**: Window functions from the `windowing` module are used in FIR coefficient design.

## References & Further Reading

- Oppenheim, A.V. and Schafer, R.W., *Discrete-Time Signal Processing*, 3rd ed., Pearson, 2009 — Chapter 7.
- Parks, T.W. and Burrus, C.S., *Digital Filter Design*, Wiley, 1987.
- Smith, S.W., *The Scientist and Engineer's Guide to Digital Signal Processing*, California Technical Publishing, 1997 — Chapters 14–16.
