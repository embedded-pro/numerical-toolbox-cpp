# Exponential Moving Average (One-Pole) — Implementation Pseudocode

> Roadmap ref: #1 (Tier 1) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class ExponentialMovingAverage:
    T     alpha        # smoothing factor in (0, 1]
    T     state        # last output y[n-1]
    bool  enabled = true
```

## Interface

```
ExponentialMovingAverage(T alpha, T initial = 0)
T    Filter(T input)         # hot path
void Reset(T value = 0)
void SetAlpha(T alpha)
void Enable() / Disable()
static T AlphaFromCutoff(T cutoffHz, T sampleRateHz)
```

## Algorithm (pseudocode)

```
function Filter(x):                       # OPTIMIZE_FOR_SPEED
    if not enabled: return x
    # y[n] = alpha*x[n] + (1 - alpha)*y[n-1]
    state = state + alpha * (x - state)   # one MAC, rearranged to avoid (1-alpha) term
    return state

function AlphaFromCutoff(fc, fs):
    # first-order RC equivalent
    dt = 1 / fs
    rc = 1 / (2*pi*fc)
    return dt / (rc + dt)
```

## Complexity & memory

- Time: `O(1)` per sample (1 multiply, 1 add/sub for the rearranged form).
- Memory: `O(1)` — a single state word plus the coefficient.

## Numerical / embedded notes

- `alpha ∈ (0,1]`; the pole `1-alpha` stays inside the unit circle ⇒ unconditionally stable.
- Rearranged update `state += alpha*(x-state)` keeps a single multiply on the hot path.
- DC gain is exactly 1 (a constant input converges to that constant).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature lets a
  `Q15`/`Q31` specialisation be added later without changing call sites.

## Deployment

- Header: `numerical/filters/passive/ExponentialMovingAverage.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class ExponentialMovingAverage<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/ExponentialMovingAverage.cpp` →
  `template class ExponentialMovingAverage<float>;`
- Test: `numerical/filters/passive/test/TestExponentialMovingAverage.cpp`
- Doc: `doc/filters/passive/ExponentialMovingAverage.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestExponentialMovingAverage.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
