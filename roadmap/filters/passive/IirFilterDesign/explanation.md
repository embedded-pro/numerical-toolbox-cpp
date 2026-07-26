# IIR Filter Design (Butterworth / Chebyshev + Bilinear) — Overview

## What it is
An on-device *design* routine: given a filter type, order, and cutoff, it computes the biquad/SOS
coefficients for a classic analog prototype (Butterworth or Chebyshev) mapped to the digital domain
by the bilinear transform.

## Why it matters (embedded)
Normally filter coefficients are baked in with a host tool like MATLAB/SciPy. Designing on the device
instead means the **cutoff and order can change at runtime** — adaptive bandwidth, user-tunable EQ,
self-calibrating instrumentation — with no host toolchain and no giant hardcoded tables.

## How it works (intuition)
Classic filters are defined by pole/zero patterns on the analog `s`-plane (Butterworth: poles evenly
spaced on a circle; Chebyshev: on an ellipse for a steeper, rippled response). The bilinear transform
warps that `s`-plane design onto the digital `z`-plane, turning it into digital coefficients. A
pre-warp step corrects the transform's frequency bending so the cutoff lands exactly where asked. The
conjugate pole/zero pairs are then grouped into second-order sections for a robust runtime filter.

## Key parameters
- **Prototype** — Butterworth (maximally flat) or Chebyshev-I (equiripple, steeper).
- **Kind** — low-pass / high-pass / band-pass / band-stop.
- **order, cutoffHz, sampleHz** — response sharpness and band edge.
- **rippleDb** — passband ripple for Chebyshev.

## Reference
T. W. Parks, C. S. Burrus, *Digital Filter Design* (1987); bilinear transform —
A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*.

## See also
`BiquadCascade` (consumes the emitted SOS coefficients), `NotchCombFilter` (band-stop special case),
`Cordic` (multiplier-free root placement).
