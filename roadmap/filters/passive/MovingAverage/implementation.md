# Moving Average (Running-Sum Boxcar) — Implementation Pseudocode

> Roadmap ref: #2 (Tier 1) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class MovingAverage:                       # N = window length
    RecursiveBuffer<T, N> window           # bounded delay line, holds x[n-1..n-N]
    T                     sum              # running sum
    T                     invN             # precomputed 1/N
    bool                  enabled = true
```

## Interface

```
MovingAverage(T initial = 0)
T    Filter(T input)          # hot path
void Reset(T value = 0)
```

## Algorithm (pseudocode)

```
function Filter(x):                        # OPTIMIZE_FOR_SPEED
    if not enabled: return x
    oldest = window.Oldest()               # x[n-N]
    sum    = sum + x - oldest              # incremental update, O(1)
    window.Push(x)                         # drop oldest, append newest
    return sum * invN                      # divide by N via multiply
```

## Complexity & memory

- Time: `O(1)` per sample — one add, one sub, one multiply, independent of `N`.
- Memory: `O(N)` for the delay line plus a single accumulator word.

## Numerical / embedded notes

- Linear phase: constant group delay of `(N-1)/2` samples; DC gain is exactly 1.
- Recursive running-sum can accumulate rounding error over long runs; periodically rebuild the
  sum from the window if drift matters.
- Optimal white-noise reducer per computation, but a poor frequency-selective filter
  (`sinc`-shaped magnitude, slow stopband roll-off).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/MovingAverage.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class MovingAverage<float, N>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/MovingAverage.cpp` →
  `template class MovingAverage<float, N>;`
- Test: `numerical/filters/passive/test/TestMovingAverage.cpp`
- Doc: `doc/filters/passive/MovingAverage.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestMovingAverage.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
