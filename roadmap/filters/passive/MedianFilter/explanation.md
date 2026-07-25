# Median Filter — Overview

## What it is
A non-linear sliding-window filter that outputs the **median** (middle value) of the last `N`
samples instead of their average.

## Why it matters (embedded)
Impulsive noise — dropped ADC samples, EMI clicks, salt-and-pepper glitches — is exactly the case
where a moving average fails: a single huge outlier drags the mean. The median ignores such
outliers entirely, using only comparisons (no multiplies), making it a cheap, robust front-end
clean-up for noisy sensor streams.

## How it works (intuition)
Sort the window and take the centre element. Because a spike lands at an *end* of the sorted order,
it never reaches the middle unless it forms a majority of the window. Step edges are preserved
because the median snaps to whichever level owns the majority of the window.

## Key parameters
- **N (window length, odd)** — larger `N` rejects wider bursts but adds `(N−1)/2` samples of lag
  and rounds off narrow legitimate peaks.

## Reference
T. Huang, G. Yang, G. Tang, "A fast two-dimensional median filtering algorithm,"
*IEEE Trans. Acoustics, Speech, and Signal Processing*, 27(1), 1979.

## See also
`MovingAverage` (linear, smears spikes), `SavitzkyGolayFilter` (peak-preserving linear),
`ExponentialMovingAverage` (cheap IIR smoother).
