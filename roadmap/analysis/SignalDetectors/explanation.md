# Signal Detectors — Overview

## What it is
A trio of tiny feature extractors that summarise a stream without storing it: a **peak-hold**
(running maximum with optional decay), a **zero-crossing counter** (sign-change events), and an
**RMS envelope** (a one-pole smoother running on `x²`).

## Why it matters (embedded)
These are the cheapest possible front-ends for higher-level logic — each is O(1) time and O(1)
memory. They power VU meters, activity/onset detection, crude pitch/frequency estimation, clipping
alarms, and wake-on-event triggers on parts with no room for an FFT.

## How it works (intuition)
- **Peak-hold** latches the largest magnitude seen; a `decay < 1` slowly releases it so a meter
  falls back after transients.
- **Zero-crossing** counting approximates fundamental frequency — a sinusoid crosses zero twice per
  period, so `f ≈ crossings / (2·T)`. A hysteresis band stops noise near zero from over-counting.
- **RMS envelope** passes `x²` through the item-1 one-pole low-pass and takes the square root,
  yielding a smooth estimate of signal power/loudness.

## Key parameters
- **decay** (peak-hold) ∈ (0, 1] — 1 latches forever, smaller releases faster.
- **hysteresis** (zero-crossing) — dead-band amplitude a crossing must exceed.
- **alpha** (RMS) — one-pole smoothing factor; smaller = smoother, slower envelope.

## Reference
R. G. Lyons, *Understanding Digital Signal Processing*, 3rd ed., Pearson, 2011.

## See also
`ExponentialMovingAverage` (the one-pole the RMS envelope reuses), `math::Statistics`,
`GoertzelAlgorithm` (precise single-tone detection).
