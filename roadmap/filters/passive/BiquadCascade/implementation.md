# Biquad / Second-Order-Section Cascade — Implementation Pseudocode

> Roadmap ref: #15 (Tier 3) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                       # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BiquadCoeffs:
    T b0, b1, b2                           # feedforward (a0-normalized)
    T a1, a2                               # feedback   (a0-normalized)

template<typename T>
class Biquad:
    BiquadCoeffs<T> c
    T z1 = 0, z2 = 0                       # Transposed Direct Form II state

template<typename T, std::size_t Sections>
class BiquadCascade:
    array<Biquad<T>, Sections> stages
```

## Interface

```
Biquad(BiquadCoeffs<T> c)
T    Filter(T x)                           # single section
BiquadCascade(array<BiquadCoeffs<T>, Sections> coeffs)
T    Filter(T x)                           # runs sections in series
void Reset()
static BiquadCoeffs<T> LowPass(T fc, T fs, T Q)          # RBJ cookbook
static BiquadCoeffs<T> HighPass / BandPass / Notch / Peaking(...)
```

## Algorithm (pseudocode)

```
function Biquad.Filter(x):                 # OPTIMIZE_FOR_SPEED  (TDF-II)
    y  = c.b0 * x + z1
    z1 = c.b1 * x - c.a1 * y + z2
    z2 = c.b2 * x - c.a2 * y
    return y

function Cascade.Filter(x):                # OPTIMIZE_FOR_SPEED
    for s in stages:
        x = s.Filter(x)
    return x

# RBJ low-pass coefficients:
w0 = 2*pi*fc/fs ; cw = cos(w0) ; alpha = sin(w0)/(2*Q)
b0=(1-cw)/2 ; b1=1-cw ; b2=(1-cw)/2 ; a0=1+alpha ; a1=-2*cw ; a2=1-alpha
divide b0,b1,b2,a1,a2 by a0
```

## Complexity & memory

- Time: `O(Sections)` per sample — 5 multiplies, 4 adds per section.
- Memory: `O(Sections)` — 2 state words + 5 coefficients per section.

## Numerical / embedded notes

- A cascade of second-order sections is far more robust than one high-order direct form: grouping
  poles/zeros into low-order sections keeps coefficient sensitivity and round-off noise small.
- Order sections by increasing peak gain and pair each pole with its nearest zero to keep
  intermediate signal levels well-scaled.
- Normalize the coefficients by `a0` offline so the hot path stays 5 multiplies + 4 adds per section.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/BiquadCascade.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template struct BiquadCoeffs<float>;` / `extern template class Biquad<float>;` /
  `extern template class BiquadCascade<float, Sections>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/BiquadCascade.cpp` →
  `template struct BiquadCoeffs<float>; template class Biquad<float>; template class BiquadCascade<float, Sections>;`
- Test: `numerical/filters/passive/test/TestBiquadCascade.cpp`
- Doc: `doc/filters/passive/BiquadCascade.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestBiquadCascade.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
