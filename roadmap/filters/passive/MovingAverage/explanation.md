# Moving Average — Overview

## What it is
A length-`N` boxcar (unweighted) FIR filter computed recursively: keep a running sum of the last
`N` samples, update it as `sum += x[n] − x[n−N]`, then divide by `N`.

## Why it matters (embedded)
For a given noise-reduction target it is the **optimal filter per unit of computation** — and the
recursive form costs `O(1)` per sample regardless of window length. Ideal for cleaning up white
noise on ADC/sensor streams when a flat passband and linear phase are wanted.

## How it works (intuition)
Averaging `N` samples attenuates zero-mean noise by `√N` while passing DC untouched. Because every
tap has equal weight, the frequency response is a `sinc` — excellent in the time domain (sharp step
response, linear phase) but a mediocre frequency-selective filter (slow roll-off, sidelobes). Its
finite memory means an impulse fully leaves after exactly `N` samples.

## Key parameters
- **N (window length)** — larger `N` ⇒ more smoothing and longer delay `(N−1)/2`.
- **initial state** — seed value to shorten the startup transient.

## Reference
S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 15
(Moving Average Filters — recursive running-sum form).

## See also
`ExponentialMovingAverage` (cheaper IIR smoother), `SavitzkyGolayFilter` (peak-preserving),
`BiquadCascade` (frequency-selective roll-off).
