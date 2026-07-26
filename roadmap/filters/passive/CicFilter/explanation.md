# CIC (Cascaded Integrator-Comb) Filter — Overview

## What it is
A multiplier-free decimation/interpolation filter built from `Stages` integrators running at the
high sample rate followed (after the rate change) by `Stages` comb (differencing) stages at the low
rate.

## Why it matters (embedded)
It is the canonical front-end for sigma-delta ADCs/DACs and any large integer rate change: it uses
**only adds, subtracts, and delays — no multipliers and no coefficient storage**. That makes it
almost free in hardware and on multiplier-poor microcontrollers.

## How it works (intuition)
An integrator is a running sum (a pole at DC); a comb with delay `M` subtracts a delayed copy (a
zero). Cascading `Stages` of each gives a `sinc^Stages` low-pass shape that suppresses the aliases
created by decimation. The trick is that integer integrator overflow wraps around and is exactly
undone by the comb subtraction, so the arithmetic stays lossless despite huge internal gain.

## Key parameters
- **R (rate change)** — decimation/interpolation factor.
- **Stages (order)** — more stages ⇒ steeper roll-off and more alias rejection, but more droop.
- **M (differential delay)** — usually 1 or 2; sets the null spacing.

## Reference
E. B. Hogenauer, "An Economical Class of Digital Filters for Decimation and Interpolation,"
*IEEE Trans. Acoustics, Speech, and Signal Processing*, 29(2), 1981.

## See also
`MovingAverage` (a 1-stage integrator-comb pair), `BiquadCascade` (droop compensation),
`IirFilterDesign` (sharper selective filtering).
