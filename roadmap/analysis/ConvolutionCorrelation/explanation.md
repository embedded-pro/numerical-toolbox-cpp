# Convolution & Correlation — Overview

## What it is
A small library of the two most fundamental DSP operations. **Convolution** slides one sequence over
another and sums the overlapping products (the time-domain action of any LTI filter). **Correlation**
does the same but without reversing one operand — it measures *similarity* as a function of lag.

## Why it matters (embedded)
Matched filtering, template matching, pulse detection, and time-delay (lag) estimation all reduce to
these two kernels. Auto-correlation also underlies pitch detection and the Yule-Walker/AR estimators
already in the library. Bounded, no-heap versions let them run on-device instead of offline.

## How it works (intuition)
Convolution output sample `n` is `Σ x[k]·h[n−k]` — flip `h`, shift it to position `n`, multiply
point-by-point, and add. Correlation is identical but with `h` *not* flipped, so a peak in the
cross-correlation marks the lag at which two signals line up best. For long signals the convolution
theorem lets you multiply spectra instead: `x∗h = IFFT(FFT(x)·FFT(h))`.

## Key parameters
- **operand lengths** — set the compile-time output size `M+K−1` (linear) or `N` (circular).
- **circular vs linear** — circular wraps modulo `N`; equal to linear only with zero-padding.
- **normalization** — divide by `r[0]` for a correlation coefficient in `[−1, 1]`.

## Reference
A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Prentice Hall, 2009,
Chs. 2 & 8.

## See also
`RealFft` / `FastFourierTransform` (fast convolution), `GoertzelAlgorithm`, `YuleWalker`
(auto-correlation-based AR estimation).
