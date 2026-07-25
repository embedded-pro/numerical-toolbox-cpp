# Biquad / Second-Order-Section Cascade — Overview

## What it is
A high-order IIR filter built as a **chain of second-order sections** ("biquads"), each with two
poles and two zeros, rather than one monolithic transfer function.

## Why it matters (embedded)
This is the highest-value passive-filter gap in the library. A single high-order direct-form IIR is
numerically fragile — small coefficient errors move poles unpredictably and round-off
can even destabilize it. Splitting into biquads is the **standard robust realization** in DSP,
giving well-conditioned coefficients and low round-off noise.

## How it works (intuition)
Any real rational transfer function factors into quadratic terms; each quadratic is one biquad.
Filtering in series applies them one after another. Because each section only ever handles two poles
and two zeros, its coefficients stay well-scaled and its sensitivity to quantization is tiny
compared with an equivalent monolithic filter. Coefficients come from closed-form design recipes
(the RBJ cookbook) or from a design routine.

## Key parameters
- **Sections (order/2)** — number of biquads; sets roll-off steepness.
- **Per-section {b0,b1,b2,a1,a2}** — derived from cutoff `fc`, sample rate `fs`, and `Q`.
- **Form** — Direct Form I (extra headroom) vs Transposed DF-II (compact state).

## Reference
A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, Ch. 6 (cascade/parallel
structures); R. Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients."

## See also
`IirFilterDesign` (generates the coefficients), `NotchCombFilter` (a specialized biquad),
`MovingAverage` (FIR alternative).
