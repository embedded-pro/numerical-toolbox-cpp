# Convolution & Correlation

## Overview & Motivation

Two signals interacting in time — one being filtered, delayed, or matched against another — are
described by convolution and correlation. Convolution is the time-domain action of any linear
time-invariant (LTI) filter: given an input signal and a system's impulse response, their
convolution is the output. Correlation measures the similarity between two signals as a function
of relative shift (lag), making it the foundation of matched filtering, pulse detection, and
time-delay estimation.

Both operations are indispensable in embedded DSP: they underlie FIR filtering, pitch detection,
radar/sonar processing, and auto-regressive modeling. Bounded, no-heap implementations make
these algorithms practical on microcontrollers.

## Mathematical Theory

### Linear Convolution

For finite sequences $x$ of length $M$ and $h$ of length $K$, the linear convolution $y = x * h$
is a sequence of length $M + K - 1$ defined by:

$$y[n] = \sum_{k=0}^{M-1} x[k] \cdot h[n - k], \quad n = 0, \ldots, M+K-2$$

where $h[j] = 0$ for $j < 0$ or $j \geq K$.

### Circular Convolution

For two sequences of equal length $N$, circular (periodic) convolution wraps indices modulo $N$:

$$y[n] = \sum_{k=0}^{N-1} x[k] \cdot h[(n - k) \bmod N], \quad n = 0, \ldots, N-1$$

Circular convolution equals linear convolution only when both operands are zero-padded to at least
$M + K - 1$ samples. Without padding, time-aliasing occurs.

### Cross-Correlation

Cross-correlation of $x$ (length $M$) and $y$ (length $K$) is defined as:

$$r_{xy}[\ell] = \sum_{n} x[n] \cdot y[n - \ell]$$

This equals linear convolution of $x$ with the time-reversed $y$:

$$r_{xy} = x * \overline{y}$$

### Auto-Correlation

Auto-correlation is cross-correlation of a signal with itself:

$$r_{xx}[\ell] = \sum_{n} x[n] \cdot x[n - \ell]$$

It is always symmetric about the zero-lag index, and its maximum occurs at zero lag.
Dividing by $r_{xx}[0]$ yields a normalized coefficient in $[-1, 1]$.

### Convolution Theorem (Fast Path)

For long signals, the convolution theorem enables an $O(L \log L)$ computation:

$$x * h = \mathcal{F}^{-1}\!\left(\mathcal{F}(x) \cdot \mathcal{F}(h)\right)$$

where $\mathcal{F}$ denotes the DFT and $L$ is the next power of two $\geq M + K - 1$. The
pointwise complex product replaces the $O(MK)$ direct sum.

## Complexity Analysis

| Operation            | Time          | Space (extra) | Notes                                 |
|----------------------|---------------|---------------|---------------------------------------|
| Linear convolution   | $O(MK)$       | $O(1)$        | Writes into caller-owned buffer       |
| Circular convolution | $O(N^2)$      | $O(1)$        | Same as linear for equal-length input |
| Cross-correlation    | $O(MK)$       | $O(K)$        | Reverses one operand on the stack     |
| Auto-correlation     | $O(M^2)$      | $O(1)$        | Alias of cross-correlation            |
| Fast convolution     | $O(L \log L)$ | $O(L)$        | $L = 2^{\lceil\log_2(M+K-1)\rceil}$   |

The fast path is beneficial when $MK > L \log_2 L$, roughly when both operands exceed 32–64
samples.

## Step-by-Step Walkthrough

Linear convolution of $x = [1, 2, 3]$ and $h = [1, 1]$ (output length $= 4$):

| $n$ | Active $k$ range | Computation             | $y[n]$ |
|-----|------------------|-------------------------|--------|
| 0   | $k=0$            | $1 \cdot 1$             | 1      |
| 1   | $k=0,1$          | $2 \cdot 1 + 1 \cdot 1$ | 3      |
| 2   | $k=1,2$          | $3 \cdot 1 + 2 \cdot 1$ | 5      |
| 3   | $k=2$            | $3 \cdot 1$             | 3      |

Result: $[1, 3, 5, 3]$.

Auto-correlation of $[1, 1, 1, 1]$ produces the triangular sequence $[1, 2, 3, 4, 3, 2, 1]$,
with the peak at the zero-lag center index (index 3 in 0-based notation).

## Pitfalls & Edge Cases

- **Time-aliasing**: Circular convolution of length $N$ equals linear convolution only when both
  operands are zero-padded to $M + K - 1 \leq N$. Failing to pad causes energy from the tail to
  wrap around and corrupt the beginning of the output.
- **Lag estimation**: `ArgMaxLag` returns the index of the maximum in the cross-correlation
  output. For cross-correlation of two length-$M$ sequences the zero-lag corresponds to index
  $M - 1$; a delay of $d$ samples appears at index $M - 1 + d$.
- **Normalization**: The raw auto-correlation at zero lag equals the signal energy. Dividing the
  entire sequence by $r[0]$ gives a coefficient bounded in $[-1, 1]$.
- **Float precision**: Accumulating many products in single precision can lose significance for
  very long sequences. The MAC loop uses the native float accumulator; for sequences longer than
  a few hundred samples, extended precision may be warranted.

## Variants & Generalizations

- **Normalized cross-correlation**: Divide $r_{xy}[\ell]$ by $\sqrt{r_{xx}[0] \cdot r_{yy}[0]}$
  for an amplitude-invariant similarity measure.
- **Overlap-add / overlap-save**: Partition long signals into short blocks and convolve each
  with $h$ via the FFT fast path, then reassemble. Reduces latency for streaming applications.
- **Partial correlation**: Compute cross-correlation over a sliding window rather than the full
  signal length — useful for non-stationary signals.

## Applications

- **FIR filtering**: Direct-form FIR is linear convolution of the input with the filter's impulse
  response.
- **Matched filtering**: Cross-correlate the received signal with a known template to detect it
  in noise.
- **Time-delay estimation (TDOA)**: The lag of the cross-correlation peak between two sensors
  gives the inter-sensor arrival time difference.
- **Pitch detection**: Auto-correlation of a speech frame reveals the fundamental period as the
  first large secondary peak.
- **AR modeling**: Yule-Walker equations use auto-correlation values to fit auto-regressive
  parameters.

## Connections to Other Algorithms

- **FastFourierTransform**: The fast convolution path is a direct application of the FFT-based
  convolution theorem.
- **GoertzelAlgorithm**: A single-frequency DFT bin — cross-correlating with a complex
  exponential at one frequency is equivalent.
- **YuleWalker**: Constructs the AR system matrix from auto-correlation lags computed by
  `AutoCorrelation`.
- **PowerSpectralDensity**: The power spectrum is the DFT of the auto-correlation sequence
  (Wiener–Khinchin theorem).

## References & Further Reading

- A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Prentice Hall,
  2009, Chapters 2 & 8.
- A. V. Oppenheim, A. S. Willsky, *Signals and Systems*, 2nd ed., Prentice Hall, 1997,
  Chapter 9.
