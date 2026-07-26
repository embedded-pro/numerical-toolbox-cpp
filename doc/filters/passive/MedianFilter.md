# Median Filter (Sliding Window)

## Overview & Motivation

Impulsive noise — EMI clicks, dropped ADC bits, salt-and-pepper sensor glitches — corrupts embedded
sensor streams in a way that linear averaging amplifies rather than rejects. A single outlier sample
at 100× the signal level drags a moving-average output for the entire window duration. The median
filter eliminates isolated spikes completely, using only comparisons and no arithmetic on the signal
values themselves, making it an ideal non-linear pre-processor for noisy ADC and sensor streams.

## Mathematical Theory

### Core Definition

Let $x[n]$ be the input sequence and $N$ the window length. Define the sorted window snapshot as
$x_{(1)} \le x_{(2)} \le \cdots \le x_{(N)}$ (order statistics). The median filter output is:

$$y[n] = x_{(\lceil N/2 \rceil)}$$

For odd $N$ this is the unique middle order statistic. For even $N$ one common convention takes the
average of the two central statistics, but this introduces arithmetic on the signal and the
associated round-off.

### Impulse Rejection Threshold

A spike is suppressed if and only if it does not form a strict majority of the window, i.e., if fewer
than $\lceil N/2 \rceil$ samples share the outlier level. An isolated impulse occupies exactly one
slot; it is always rejected for $N \ge 3$.

### Step-Edge Preservation

A step transition from level $a$ to level $b$ causes the median to switch from $a$ to $b$ after
exactly $\lfloor N/2 \rfloor$ samples of delay — without overshoot, Gibbs ringing, or smearing. This
is the fundamental advantage over linear filters: the nonlinearity of the sort operation preserves
edges while rejecting impulses.

### Lag

The causal sliding-window median introduces a latency of $\lfloor (N-1)/2 \rfloor$ samples, which
is the same group delay as a symmetric FIR of the same length. Unlike a linear FIR, however, the
delay is signal-dependent near edges.

## Algorithm

### Insertion-Sort Approach

For small, fixed window sizes $N$ (typically 3–9 in embedded use), copying the ring buffer into a
scratch array and applying insertion sort is cache-friendly and avoids heap allocation. The
comparison count is at most $N(N-1)/2$ — fully bounded by the compile-time constant $N$.

Insertion sort on the scratch copy:

$$\text{for } i = 1, \ldots, N-1: \text{ shift } \text{scratch}[0 \ldots i-1] \text{ right until sorted position for scratch}[i]$$

The median is then $\text{scratch}[\lfloor N/2 \rfloor]$.

### Huang Histogram Variant

Huang, Yang, and Tang (1979) describe an $O(1)$-per-sample update based on a running histogram: the
departing sample decrements its histogram bin and the arriving sample increments its bin, then the
median bin is walked left or right by one position. This reduces per-sample work from $O(N)$ to
$O(1)$ but requires a histogram array indexed over the signal alphabet — impractical for
floating-point streams on embedded targets.

## Complexity Analysis

| Variant              | Time per sample | Space   | Notes                                      |
|----------------------|-----------------|---------|--------------------------------------------|
| Insertion sort (used) | $O(N)$          | $O(N)$ | Optimal for small fixed $N$; no heap       |
| Huang histogram      | $O(1)$          | $O(K)$ | $K$ = alphabet size; integer signals only  |

For $N \le 9$ the insertion-sort inner loop is unrolled by the compiler and runs in a handful of
compare-and-swap operations — faster in practice than histogram maintenance.

## Step-by-Step Walkthrough

Input: $x = [0.1, 0.1, 0.9, 0.1, 0.1]$, window size $N = 3$, initial window pre-seeded with $0.0$.

| $n$ | $x[n]$ | Window (ring buffer) | Sorted scratch          | $y[n]$ |
|-----|--------|----------------------|-------------------------|--------|
| 0   | 0.1    | [0.0, 0.0, 0.1]      | [0.0, **0.0**, 0.1]     | 0.0    |
| 1   | 0.1    | [0.0, 0.1, 0.1]      | [0.0, **0.1**, 0.1]     | 0.1    |
| 2   | 0.9    | [0.1, 0.1, 0.9]      | [0.1, **0.1**, 0.9]     | 0.1    |
| 3   | 0.1    | [0.1, 0.9, 0.1]      | [0.1, **0.1**, 0.9]     | 0.1    |
| 4   | 0.1    | [0.9, 0.1, 0.1]      | [0.1, **0.1**, 0.9]     | 0.1    |

The bold value is the median at index $\lfloor 3/2 \rfloor = 1$. The spike at $n = 2$ ($x = 0.9$)
appears in only one of the three window slots at any given time, so it is never the majority and
never reaches the output.

## Pitfalls & Edge Cases

- **Even window length**: the two central order statistics may differ, requiring an average; use odd
  $N$ to keep the output comparison-only.
- **Window width vs. pulse width**: an impulse burst wider than $\lfloor N/2 \rfloor$ samples is
  not rejected; $N$ must be at least $2 \times \text{burst\_width} + 1$.
- **Startup / warm-up**: pre-seeding the window with an initial value prevents undefined output
  during the first $N-1$ samples.
- **Signal plateaus**: when the signal is constant at the impulse level for more than half the
  window, that value becomes the majority and passes through — this is expected behaviour and
  documents the $\lfloor N/2 \rfloor$ rejection limit.
- **Latency vs. rejection trade-off**: increasing $N$ widens the rejection band but also increases
  latency and rounds off narrow legitimate peaks.

## Variants & Generalizations

- **Weighted median**: assigns integer multiplicities to samples; shifts the breakdown point without
  changing the comparison-only property.
- **Running-histogram median (Huang 1979)**: $O(1)$ update for integer signals over a bounded
  alphabet.
- **Two-pass 2-D median**: Huang's paper targets image processing; separable row–column passes
  generalize to 2-D arrays.
- **Recursive median**: feeds filtered output back into the window; achieves stronger smoothing at
  the cost of potential limit-cycle oscillations.

## Applications

- ADC spike rejection before integration or threshold detection.
- Salt-and-pepper denoising in IMU/MEMS sensor streams.
- Pre-filter before a PID controller to prevent derivative kick from glitches.
- Image-row processing on embedded vision pipelines (1-D median per scan line).

## Connections to Other Algorithms

- `MovingAverage` is the linear equivalent; it smears both noise and edges, while the median
  preserves edges at the cost of being nonlinear and harder to analyse in the frequency domain.
- `ExponentialMovingAverage` provides even lower memory cost but is sensitive to impulsive outliers.
- `FirFilter` with optimal coefficients can achieve equiripple passband/stopband behaviour but
  cannot reject isolated spikes without distorting step edges.

## References & Further Reading

- T. Huang, G. Yang, G. Tang, "A fast two-dimensional median filtering algorithm," *IEEE
  Transactions on Acoustics, Speech, and Signal Processing*, 27(1), pp. 13–18, 1979.
- A. V. Oppenheim & R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., §11.4 — Median and
  Order-Statistic Filters.
- L. Yin, R. Yang, M. Gabbouj, Y. Neuvo, "Weighted median filters: a tutorial," *IEEE Transactions
  on Circuits and Systems II*, 43(3), 1996.
