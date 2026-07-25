# Saturation / Rate-Limiter / Slew Blocks — Implementation Pseudocode

> Roadmap ref: #3 (Tier 1) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class Saturation:
    T lo, hi                # output bounds, lo < hi

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class RateLimiter:
    T     maxRate           # |Δu| per second (slew rate)
    T     sampleTime        # Ts in seconds
    T     previous          # last emitted output u[n-1]
    bool  primed = false    # first sample seeds `previous`

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class SlewLimitedSaturation:         # composition of the two
    Saturation<T>   clampBlock
    RateLimiter<T>  slewBlock
```

## Interface

```
Saturation(T lo, T hi)
T    Clamp(T u) const

RateLimiter(T maxRate, T sampleTime)
T    Limit(T u)                      # hot path
void Reset(T value = 0)

SlewLimitedSaturation(T lo, T hi, T maxRate, T sampleTime)
T    Apply(T u)                      # slew first, then clamp
```

## Algorithm (pseudocode)

```
function Clamp(u):                    # OPTIMIZE_FOR_SPEED
    return min(max(u, lo), hi)

function Limit(u):                    # OPTIMIZE_FOR_SPEED
    if not primed:
        previous = u; primed = true
        return u
    step  = maxRate * sampleTime      # max change this tick
    delta = clamp(u - previous, -step, +step)
    previous = previous + delta
    return previous

function Apply(u):
    return clampBlock.Clamp(slewBlock.Limit(u))
```

## Complexity & memory

- Time: `O(1)` per sample — two compares (clamp) plus one compare-pair (slew).
- Memory: `O(1)` — bounds, rate, `Ts`, and one state word per limiter.

## Numerical / embedded notes

- Precompute `step = maxRate*sampleTime` once if `Ts` is fixed to save a multiply per tick.
- `min`/`max` are branch-light and keep the hot path predictable.
- Order matters: slew-then-clamp guarantees the output honours both bounds; clamp-then-slew can
  leave the output outside `[lo,hi]` for one tick.
- These blocks are the anti-windup prerequisite: feed the clamped output back to the integrator
  (see `PidIncremental`) so it stops winding while the actuator is pinned.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/SaturationRateLimiter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Clamp`/`Limit`/`Apply`, and
  `extern template class Saturation<float>; RateLimiter<float>; SlewLimitedSaturation<float>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/SaturationRateLimiter.cpp` →
  `template class Saturation<float>; template class RateLimiter<float>; template class SlewLimitedSaturation<float>;`
- Test: `numerical/controllers/implementations/test/TestSaturationRateLimiter.cpp`
- Doc: `doc/controllers/SaturationRateLimiter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSaturationRateLimiter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
