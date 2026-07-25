# LMS / NLMS Adaptive Filter — Overview

## What it is
An adaptive FIR filter that **learns its own coefficients** online. Each sample it compares its
output to a desired signal and nudges the weights down the instantaneous error gradient:
`w ← w + μ·e·x`. NLMS (Normalized LMS) divides the step by the input energy `‖x‖²` so the learning
rate no longer depends on signal level.

## Why it matters (embedded)
It is the workhorse of real-time adaptation: **acoustic echo cancellation**, adaptive noise
cancellation, channel equalization, active vibration/noise control, and online system
identification. Compared with Recursive Least Squares it needs no covariance matrix — just two
vectors — so it fits tiny MCUs and tracks slowly changing environments in `O(Taps)` per sample.

## How it works (intuition)
Think of gradient descent applied one sample at a time. The error `e = desired − wᵀx` measures how
wrong the current weights are; scaling the input `x` by that error points each weight in the
direction that reduces it. A small step `μ` averages out noise but adapts slowly; a large step
adapts fast but risks instability. NLMS auto-scales the step so loud and quiet signals adapt alike.

## Key parameters
- **μ (step size / learning rate)** — the speed-vs-stability knob; bounded by input power (LMS) or ~2 (NLMS).
- **Tap count** — length of the FIR; must span the impulse response being modeled.
- **Normalization / ε** — enable NLMS for level-independent convergence; ε guards against silence.

## Reference
B. Widrow, M. E. Hoff, "Adaptive switching circuits," *IRE WESCON Conv. Rec.*, 1960;
S. Haykin, *Adaptive Filter Theory*.

## See also
`RecursiveLeastSquares` (faster-converging, heavier online estimator), `Fir` (fixed-coefficient
filter), `ConvolutionCorrelation`.
