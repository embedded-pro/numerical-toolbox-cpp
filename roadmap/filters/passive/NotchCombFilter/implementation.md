# Notch / Comb Filter — Implementation Pseudocode

> Roadmap ref: #16 (Tier 3) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                       # static_assert(std::is_floating_point_v<T>); instantiated for float
class NotchFilter:                         # single-frequency biquad notch
    BiquadCoeffs<T> c
    T z1 = 0, z2 = 0                        # TDF-II state

template<typename T, std::size_t D>
class CombFilter:                          # periodic notches at k·fs/D
    RecursiveBuffer<T, D> delay             # D-sample delay line
    T    gain                               # 0 ≤ gain < 1
    bool feedback = false                   # false = FIR comb, true = IIR comb
```

## Interface

```
NotchFilter(T f0, T fs, T Q)               # RBJ notch coefficients
T    Filter(T x)
CombFilter(T gain, bool feedback = false)
T    Filter(T x)
void Reset()
```

## Algorithm (pseudocode)

```
# RBJ notch biquad coefficients (zeros ON the unit circle at ±w0):
w0 = 2*pi*f0/fs ; cw = cos(w0) ; alpha = sin(w0)/(2*Q)
b0=1 ; b1=-2*cw ; b2=1 ; a0=1+alpha ; a1=-2*cw ; a2=1-alpha
divide b0,b1,b2,a1,a2 by a0

function Notch.Filter(x):                   # OPTIMIZE_FOR_SPEED  (TDF-II)
    y  = b0*x + z1
    z1 = b1*x - a1*y + z2
    z2 = b2*x - a2*y
    return y

function Comb.Filter(x):                    # OPTIMIZE_FOR_SPEED
    d = delay.Oldest()                      # sample D steps ago
    if not feedback:                        # feedforward (FIR) comb
        y = x - gain * d ; delay.Push(x)
    else:                                   # feedback (IIR) comb
        y = x + gain * d ; delay.Push(y)
    return y
```

## Complexity & memory

- Notch: `O(1)` per sample (one biquad); memory `O(1)`.
- Comb: `O(1)` per sample; memory `O(D)` for the delay line.

## Numerical / embedded notes

- Notch width is set by `Q`: higher `Q` ⇒ narrower notch, but poles sit closer to the unit circle
  ⇒ more coefficient sensitivity and a longer ring-down.
- Feedback comb with `gain → 1` places poles near the unit circle → conditionally stable; keep
  `gain < 1` with margin to avoid limit cycles / sustained oscillation.
- `D = round(fs / f0)`; if `fs` is not an integer multiple of the line frequency the comb is
  detuned and leaves residual hum — prefer a sample rate that divides evenly, or cascade single
  notches.
- Cascade a `50/60 Hz` notch with a comb to also kill the harmonics; notch DC gain is 1.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/NotchCombFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class NotchFilter<float>;` / `extern template class CombFilter<float, D>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/NotchCombFilter.cpp` →
  `template class NotchFilter<float>; template class CombFilter<float, D>;`
- Test: `numerical/filters/passive/test/TestNotchCombFilter.cpp`
- Doc: `doc/filters/passive/NotchCombFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestNotchCombFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
