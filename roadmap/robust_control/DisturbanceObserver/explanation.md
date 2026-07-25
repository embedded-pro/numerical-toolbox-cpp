# Disturbance Observer (DOB) — Overview

## What it is
A bolt-on estimator that lumps everything the nominal model does not explain — external loads,
friction, parameter drift — into a single **equivalent disturbance**, estimates it, and subtracts it
from the control. It wraps around an existing loop without redesigning the nominal controller.

## Why it matters (embedded)
DOB buys **strong disturbance rejection for almost free**. You keep the controller you already tuned
and add an inner correction that makes the real (messy) plant behave like the clean nominal model.
That is enormously valuable on microcontrollers where redesigning or re-identifying the plant in the
field is impractical — motion stages, drives, and mechatronics use it everywhere.

## How it works (intuition)
If you knew the disturbance, you would just cancel it. You cannot measure it, so you *infer* it:
pass the measured output through the **inverse of the nominal plant** to reconstruct what input
"must" have produced it, subtract the input you actually applied, and the difference is the
disturbance. That inverse is improper and noise-sensitive, so it is cascaded with a low-pass
**Q-filter** that makes it realizable and sets the frequency band over which the DOB is active.
Inside the Q-band the plant looks nominal; outside it the DOB politely gets out of the way.

## Key parameters
- **nominal plant `Pn`** — the model whose inverse reconstructs the disturbance.
- **Q-filter** — low-pass that bounds the rejection bandwidth and makes `Q·Pn^{-1}` proper; unity DC gain rejects constant loads.
- **Q relative degree** — must be ≥ `Pn`'s so the realization does not differentiate noise.

## Reference
W.-H. Chen, J. Yang, L. Guo, S. Li, "Disturbance-Observer-Based Control and Related Methods — An
Overview," *IEEE Trans. Industrial Electronics*, 63(2), 2016.

## See also
`BiquadCascade` (the Q-filter); `LuenbergerObserver` (state-space estimation sibling);
`ActiveDisturbanceRejectionControl` (estimates the disturbance as an augmented state instead).
