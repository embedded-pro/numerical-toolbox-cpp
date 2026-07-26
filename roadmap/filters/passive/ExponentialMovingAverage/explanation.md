# Exponential Moving Average — Overview

## What it is
A single-pole recursive low-pass filter (a.k.a. leaky integrator). Each output is a weighted
blend of the newest sample and the previous output: `y[n] = α·x[n] + (1−α)·y[n−1]`.

## Why it matters (embedded)
It is the cheapest useful smoother in existence — **one multiply, one add, one state word**.
Ubiquitous for de-noising sensor readings, tracking a slowly-moving DC level, and computing
running averages where a full moving-average buffer would cost too much RAM.

## How it works (intuition)
`α` sets the trade-off between smoothing and responsiveness. Small `α` → heavy smoothing, slow
to react; large `α` → light smoothing, fast to react. The filter has an exponentially-decaying
impulse response, hence the name. Its `−3 dB` cutoff maps directly to `α` via the first-order
RC equivalence.

## Key parameters
- **α (smoothing factor)** ∈ (0, 1] — or equivalently a cutoff frequency via `AlphaFromCutoff`.
- **initial state** — seed value to avoid a startup transient.

## Reference
S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 19
(Recursive / single-pole filters).

## See also
`MovingAverage` (linear-phase alternative), `BiquadCascade` (higher-order roll-off).
