# Decibels & Magnitude-Response Helpers

## Overview & Motivation

Audio, filter, and control-system specifications express signal levels and filter performance in decibels (dB) because the human auditory system and most engineering metrics scale logarithmically with amplitude ratio. Specifying a stop-band attenuation of 60 dB or a pass-band ripple of 0.1 dB is natural and compact; the equivalent linear ratios (1 000 : 1 and 1.01161 : 1) are not. A small set of conversion primitives — `ToDecibels`, `FromDecibels`, and two derived helpers for attenuation and ripple — centralises this conversion and eliminates scattered, error-prone inline `20·log10` expressions throughout the rest of the library.

## Mathematical Theory

### Magnitude Decibel Conversion

For a positive amplitude ratio $r > 0$, the equivalent level in decibels is:

$$L_{\mathrm{dB}} = 20 \log_{10}(r)$$

The factor 20 (rather than 10) reflects the voltage/pressure convention: power is proportional to the square of amplitude, so a doubling of amplitude ($r = 2$) gives a 6 dB increase, matching the $10 \log_{10}(4) = 6.02$ dB power equivalent.

### Inverse Conversion

$$r = 10^{L_{\mathrm{dB}}/20}$$

This inverse is exact for all finite $L_{\mathrm{dB}}$; no guard is needed on the output side.

### Zero and Negative Input Guard

$\log_{10}(0) = -\infty$; negative ratios are physically meaningless. Both cases are mapped to a finite floor value $L_{\min}$ chosen well below any engineering specification of interest:

$$L_{\mathrm{dB}} = \max\!\left(20\log_{10}(r),\; L_{\min}\right), \quad r > 0$$
$$L_{\mathrm{dB}} = L_{\min}, \quad r \leq 0$$

A floor of $-160\,\mathrm{dB}$ corresponds to an amplitude ratio below $10^{-8}$, safely beyond the dynamic range of any practical floating-point computation in 32-bit single precision.

### Derived Helpers

**Stop-band attenuation** between a pass-band ratio $r_p$ and a stop-band ratio $r_s$:

$$A = L_{\mathrm{dB}}(r_p) - L_{\mathrm{dB}}(r_s)$$

**Pass-band ripple** between the maximum and minimum in-band ratios $r_{\max}$ and $r_{\min}$:

$$\Delta = L_{\mathrm{dB}}(r_{\max}) - L_{\mathrm{dB}}(r_{\min})$$

Both are simple differences in decibel space, exploiting the logarithm identity $\log(a/b) = \log a - \log b$.

## Complexity Analysis

| Operation       | Time     | Space | Notes                              |
|-----------------|----------|-------|------------------------------------|
| `ToDecibels`    | O(1)     | O(1)  | One `log10` + one `max` + one `mul` |
| `FromDecibels`  | O(1)     | O(1)  | One `pow`                          |
| `AttenuationDb` | O(1)     | O(1)  | Two `ToDecibels` + one subtraction |
| `RippleDb`      | O(1)     | O(1)  | Two `ToDecibels` + one subtraction |

No state, no buffers. All operations are pure functions.

## Step-by-Step Walkthrough

Converting a ratio of 10 to decibels:

1. Input $r = 10$; guard passes ($r > 0$).
2. Compute $20 \cdot \log_{10}(10) = 20 \cdot 1 = 20$.
3. Apply floor: $\max(20, -160) = 20$.
4. Output: $20\,\mathrm{dB}$.

Round-trip for $r = 0.5$:

1. `ToDecibels(0.5)` = $20 \cdot \log_{10}(0.5) \approx -6.0206\,\mathrm{dB}$.
2. `FromDecibels(-6.0206)` = $10^{-6.0206/20} \approx 0.5$.

## Pitfalls & Edge Cases

- Passing $r = 0$ produces $-\infty$ from `log10`; the floor guard prevents propagation into downstream computations.
- Negative ratios indicate a programming error (signed sample values must not be passed directly as ratios without taking absolute value first); they are silently floored rather than raising an exception, consistent with the no-exception policy.
- With `fast-math` enabled, the compiler may fuse or reorder floating-point operations. The `log10` result is still monotone and the floor remains correct because it uses `std::max`, which is not reordered away.
- `FromDecibels` has no floor; at very large positive dB values the result overflows to `+inf` in float — this is expected behaviour for out-of-range inputs.

## Variants & Generalizations

- Power decibels use $10 \log_{10}(\cdot)$ (factor 10 rather than 20). The amplitude convention used here ($\times 20$) is correct for voltage, pressure, and filter transfer-function magnitude.
- Field-quantity vs. power-quantity disambiguation: IEEE 60268 / IEC 61672 mandate $20 \log_{10}$ for sound pressure level; the same convention applies to filter magnitude response.
- The floor can be parameterised if a stricter or looser sentinel is required; the default of $-160\,\mathrm{dB}$ is conservative for 32-bit float.

## Applications

- Filter specification: stop-band attenuation and pass-band ripple in dB are the primary acceptance criteria for IIR/FIR designs.
- Frequency response plots: `FrequencyResponse::Calculate()` already returns magnitude in dB using $20 \log_{10}$; these helpers provide the same conversion for ad-hoc analysis.
- Controller gain margin is expressed in dB; converting from a linear ratio with `ToDecibels` avoids duplication.
- Audio dynamic processing (compressor thresholds, limiter ceilings) and acoustic measurement both use dB natively.

## Connections to Other Algorithms

- `control_analysis::FrequencyResponse` internally applies $20 \log_{10}(\|H\|)$ on its magnitude output vector; these helpers are the scalar equivalent exposed for library consumers.
- Pass-band ripple computed by `RippleDb` feeds directly into filter-design acceptance testing alongside the step/transient-response metrics in the evaluation primitives family.

## References & Further Reading

- Proakis, J. & Manolakis, D., "Digital Signal Processing", 4th ed., Prentice Hall, 2007 — Appendix A (decibel notation).
- Zolzer, U., "DAFX: Digital Audio Effects", 2nd ed., Wiley, 2011 — Chapter 2 (level and gain in dB).
- IEC 61672-1:2013, "Electroacoustics — Sound level meters — Part 1: Specifications."
