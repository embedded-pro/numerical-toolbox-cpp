# Goertzel Algorithm — Implementation Pseudocode

> Roadmap ref: #13 (Tier 2) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class Goertzel:
    T      coeff        # 2·cos(2π·k/N), precomputed
    T      s1, s2       # recurrence state: s[n-1], s[n-2]
    T      cosine       # cos(2π·k/N), for complex output
    T      sine         # sin(2π·k/N)
    size_t N            # block length
    size_t n            # samples fed so far
```

## Interface

```
Goertzel(size_t k, size_t N)                  # target bin k of an N-point DFT
Goertzel(T targetHz, T sampleHz, size_t N)    # convenience: pick the nearest bin
void Push(T x)                                # hot path: one sample into the recurrence
bool Ready() const                            # true after N samples
T    Magnitude()                              # |X[k]|
math::Complex<T> Result()                     # real + imaginary X[k]
void Reset()
static T Coefficient(size_t k, size_t N)      # 2·cos(2π·k/N)
```

## Algorithm (pseudocode)

```
function Push(x):                             # OPTIMIZE_FOR_SPEED
    s0 = x + coeff * s1 - s2                  # second-order recurrence
    s2 = s1
    s1 = s0
    n += 1

function Result():                            # call once, after N samples
    real =  s1 - s2 * cosine
    imag =        s2 * sine
    return Complex(real, imag)

function Magnitude():
    # avoids the two trig mults: |X|^2 = s1^2 + s2^2 - coeff·s1·s2
    return sqrt(s1*s1 + s2*s2 - coeff*s1*s2)
```

## Complexity & memory

- Time: `O(N)` total — **one multiply and two adds per sample**; trig only at the end.
- Memory: `O(1)` — two state words, no input buffer, no twiddle table.
- Roughly `N/log N` cheaper than an FFT when only a handful of bins are needed.

## Numerical / embedded notes

- `coeff = 2·cos(2π·k/N)` is `constexpr`/precomputed; only `k` and `N` are needed at runtime.
- The recurrence pole sits **on** the unit circle, so it is marginally stable — keep blocks finite
  (`N` samples) and `Reset()` between blocks to stop error accumulation.
- Prefer the magnitude-squared form `s1²+s2²−coeff·s1·s2` to skip the final `sqrt` on the hot path.
- Bin resolution is `fs/N`; choose `N` so the tone of interest lands near an integer `k` for best SNR.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/GoertzelAlgorithm.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Push`, and
  `extern template class Goertzel<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/GoertzelAlgorithm.cpp` → `template class Goertzel<float>;`
- Test: `numerical/analysis/test/TestGoertzelAlgorithm.cpp`
- Doc: `doc/analysis/GoertzelAlgorithm.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestGoertzelAlgorithm.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
