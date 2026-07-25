# IIR Filter Design (Butterworth / Chebyshev + Bilinear) — Implementation Pseudocode

> Roadmap ref: #45 (Tier 5) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t MaxOrder>           # static_assert(std::is_floating_point_v<T>); instantiated for float
class IirFilterDesign:
    array<BiquadCoeffs<T>, (MaxOrder+1)/2> sections
    count sectionCount = 0
```

## Interface

```
enum class Prototype { Butterworth, ChebyshevI }
enum class Kind      { LowPass, HighPass, BandPass, BandStop }

IirFilterDesign()
count Design(Prototype p, Kind k, int order,
             T cutoffHz, T sampleHz, T rippleDb = 0)
BiquadCoeffs<T> Section(int i) const                  # feed into BiquadCascade (#15)
```

## Algorithm (pseudocode)

```
function Design(proto, kind, order, fc, fs, ripple):
    # 1. analog prototype poles on the s-plane (normalized cutoff = 1)
    if proto == Butterworth: poles = EquallySpacedOnLeftSemicircle(order)
    else:                    poles = OnEllipse(order, ripple)      # Chebyshev-I

    # 2. pre-warp the cutoff so the bilinear map lands exactly on fc
    wc = 2*fs * tan(pi * fc / fs)

    # 3. analog frequency transform LP -> {LP, HP, BP, BS}, scaled by wc
    (poles, zeros, gain) = AnalogTransform(poles, kind, wc)

    # 4. bilinear transform each root:  s -> (2/Ts)*(z-1)/(z+1)
    Ts = 1 / fs
    for r in poles ∪ zeros:  r_z = (1 + r*Ts/2) / (1 - r*Ts/2)

    # 5. pair complex-conjugate roots into second-order sections
    sectionCount = GroupConjugatePairs(poles_z, zeros_z, gain) -> sections
    return sectionCount
```

## Complexity & memory

- Time: `O(order)` one-off design work (root placement, transform, pairing); no per-sample cost —
  runtime filtering is `BiquadCascade`'s `O(Sections)`.
- Memory: `O(order)` emitted coefficients.

## Numerical / embedded notes

- Always pre-warp: the bilinear transform is non-linear in frequency, so
  `wc = 2·fs·tan(π·fc/fs)` is required for the digital cutoff to land on `fc`.
- Emit **second-order sections**, never one high-order polynomial, so the runtime filter stays
  well-conditioned.
- Butterworth = maximally flat passband; Chebyshev-I = equiripple passband with steeper roll-off for
  the same order (`rippleDb` sets the ripple).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/IirFilterDesign.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Design`, and
  `extern template class IirFilterDesign<float, MaxOrder>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/IirFilterDesign.cpp` →
  `template class IirFilterDesign<float, MaxOrder>;`
- Test: `numerical/filters/passive/test/TestIirFilterDesign.cpp`
- Doc: `doc/filters/passive/IirFilterDesign.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestIirFilterDesign.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
