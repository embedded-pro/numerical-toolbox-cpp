# Gain-Scheduled Controller — Implementation Pseudocode

> Roadmap ref: #10 (Tier 2) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t GainSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SchedulePoint:
    T                        breakpoint      # scheduling-variable value
    std::array<T, GainSize>  gains           # controller gains at this point

template<typename T, std::size_t N, std::size_t GainSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class GainScheduledController:
    std::array<SchedulePoint<T,GainSize>, N> table   # sorted by breakpoint, ascending
    std::array<T, GainSize>                  active   # last interpolated gain set
```

## Interface

```
GainScheduledController(std::array<SchedulePoint<T,GainSize>, N> table)
const std::array<T,GainSize>& Schedule(T schedulingVariable)   # hot path
const std::array<T,GainSize>& ActiveGains() const
```

## Algorithm (pseudocode)

```
function Schedule(s):                          # OPTIMIZE_FOR_SPEED
    # 1. Saturate to table range (hold end gains outside the grid)
    if s <= table[0].breakpoint:   return (active = table[0].gains)
    if s >= table[N-1].breakpoint: return (active = table[N-1].gains)

    # 2. Locate bracketing interval [i, i+1]
    i = FindInterval(table, s)                 # linear or binary search on breakpoints

    # 3. Linear interpolation weight within the interval
    lo = table[i].breakpoint;  hi = table[i+1].breakpoint
    w  = (s - lo) / (hi - lo)                  # w in [0,1]

    # 4. Blend each gain component
    for k in 0..GainSize-1:
        active[k] = table[i].gains[k] + w * (table[i+1].gains[k] - table[i].gains[k])
    return active
```

## Complexity & memory

- Time: `O(log N)` (binary search) or `O(N)` (linear scan) plus `O(GainSize)` blend per update.
- Memory: `O(N·GainSize)` for the table, `O(GainSize)` for the active gains — all static.

## Numerical / embedded notes

- Breakpoints must be **strictly increasing**; assert this once at construction.
- Endpoint saturation (clamp-and-hold) avoids extrapolation, which is unsafe for control gains.
- The blend uses the rearranged form `a + w·(b−a)` (one multiply) to stay accurate near `w≈0`,
  mirroring the EMA update.
- Gains are only interpolated here, never used to drive dynamics — the caller applies them to its
  own control law (dependency inversion keeps this block plant-agnostic).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/GainScheduledController.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Schedule`, and
  `extern template class GainScheduledController<float, N, GainSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/GainScheduledController.cpp` →
  `template class GainScheduledController<float, N, GainSize>;`
- Test: `numerical/controllers/implementations/test/TestGainScheduledController.cpp`
- Doc: `doc/controllers/GainScheduledController.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestGainScheduledController.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
