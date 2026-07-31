# Signal Detectors

## Overview & Motivation

Real-time signal monitoring requires lightweight, stateful primitives that track envelope, periodicity, and energy without buffering entire waveforms. Three composable detectors cover the most common needs: holding the peak magnitude for VU-style metering, counting sign reversals to estimate frequency, and smoothing instantaneous power to approximate RMS level. All three operate in O(1) time and O(1) memory per sample, making them suitable for tight interrupt service routines.

## Mathematical Theory

### Peak Hold

Let $m_n = |x_n|$. With a decay coefficient $d \in (0, 1]$, the held peak evolves as:

$$p_n = \max(m_n,\; d \cdot p_{n-1})$$

When $d = 1$ the peak is infinite-hold; when $d < 1$ it releases exponentially toward zero between new maxima.

### Zero-Crossing Detection

A zero crossing occurs at sample $n$ when the sign of $x_n$ differs from the sign of $x_{n-1}$ and $|x_n|$ exceeds a hysteresis threshold $h \geq 0$:

$$\text{crossed}_n = \bigl(\mathrm{sgn}(x_n) \neq \mathrm{sgn}(x_{n-1})\bigr) \land \bigl(|x_n| > h\bigr)$$

The instantaneous fundamental frequency of a periodic signal can be estimated from the cumulative count $C$ over a window of $N$ samples at sampling period $T_s$:

$$f_0 \approx \frac{C}{2 \, N \, T_s}$$

### RMS Envelope

A one-pole IIR filter is applied to $x^2$, yielding an exponentially weighted mean-square:

$$E_n = E_{n-1} + \alpha\,(x_n^2 - E_{n-1}), \quad \alpha \in (0, 1)$$

The RMS envelope is:

$$r_n = \sqrt{E_n}$$

This is structurally identical to an exponential moving average applied to the squared signal.

## Complexity Analysis

| Detector              | Time per sample                | State words         |
|-----------------------|--------------------------------|---------------------|
| Peak Hold             | O(1) — 1 mul, 1 compare, 1 abs | 1 scalar            |
| Zero-Crossing Counter | O(1) — 1 compare, 1 abs        | 1 scalar + 1 uint32 |
| RMS Envelope          | O(1) — 1 MAC + 1 sqrt          | 1 scalar            |

No buffers, no windows, no dynamic allocation.

## Step-by-Step Walkthrough

### Peak Hold (decay = 0.5), input = [0.8, 0, 0, 0]

| n | $x_n$ | $m_n$ | $d \cdot p_{n-1}$ | $p_n$ |
|---|-------|-------|-------------------|-------|
| 0 | 0.8   | 0.8   | 0.0               | 0.8   |
| 1 | 0.0   | 0.0   | 0.4               | 0.4   |
| 2 | 0.0   | 0.0   | 0.2               | 0.2   |
| 3 | 0.0   | 0.0   | 0.1               | 0.1   |

### Zero-Crossing Counter (h = 0), input = [1, -1, 1, -1]

| n | $x_n$ | prev sign | curr sign | crossed | C |
|---|-------|-----------|-----------|---------|---|
| 0 | +1    | 0         | +         | no      | 0 |
| 1 | -1    | +         | -         | yes     | 1 |
| 2 | +1    | -         | +         | yes     | 2 |
| 3 | -1    | +         | -         | yes     | 3 |

### RMS Envelope ($\alpha = 0.25$), DC input $x = 0.4$

$E_0 = 0$; after $n$ steps, $E_n = 0.16\,(1-(1-0.25)^n)$. At $n = 20$: $E_{20} \approx 0.1599$, $r_{20} \approx 0.3999$.

## Pitfalls & Edge Cases

- Decay = 0 collapses the peak hold to the instantaneous absolute value; decay must be in $(0, 1]$ for meaningful hold behavior.
- Zero-crossing sign comparison must treat $x = 0$ consistently; a sample at exactly zero with $h = 0$ can cause spurious double-counts if not handled. The hysteresis guard mitigates this.
- RMS Envelope with very small $\alpha$ responds slowly to changes in signal level; with $\alpha$ near 1 it approaches the instantaneous absolute value squared.
- The RMS detector returns 0 at initialization ($E_0 = 0$) and converges to the true RMS from below for positive-power signals.

## Variants & Generalizations

- The peak hold decay parameter maps directly to a time constant: $\tau = -T_s / \ln(d)$, bridging the gap between digital and analog VU meter release times.
- A bilateral zero-crossing counter can distinguish positive-going from negative-going edges, enabling half-wave frequency estimation.
- The RMS envelope is the special case of an exponential moving average applied to $x^2$; replacing the squaring with $|x|$ yields a mean-absolute-value envelope.

## Applications

- VU meters and compressor side-chains use peak hold with a configurable release time.
- Zero-crossing rate is a low-cost voiced/unvoiced classifier in speech processing.
- RMS envelope tracks AC signal levels in power monitoring, audio dynamic processing, and vibration analysis.
- All three detectors compose naturally into a signal statistics block for real-time diagnostics.

## Connections to Other Algorithms

- RMS Envelope is structurally equivalent to the Exponential Moving Average (EMA) filter applied to $x^2$, sharing the same one-pole recurrence.
- Zero-crossing detection is a degenerate form of edge detection in discrete sequences, related to sign-change detection in root-finding algorithms.

## References & Further Reading

- Zolzer, U., "DAFX: Digital Audio Effects", 2nd ed., Wiley, 2011 — Chapter 2 (dynamics processing, envelope followers).
- Proakis, J. & Manolakis, D., "Digital Signal Processing", 4th ed., Prentice Hall, 2007 — Section 1.4 (zero-crossing rate).
- Smith, J.O., "Introduction to Digital Filters with Audio Applications", W3K Publishing, 2007 — [online](https://ccrma.stanford.edu/~jos/filters/).
