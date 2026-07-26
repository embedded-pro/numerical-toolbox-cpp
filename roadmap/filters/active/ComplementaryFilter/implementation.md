# Complementary Filter — Implementation Pseudocode

> Roadmap ref: #9 (Tier 2) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                       # static_assert(std::is_floating_point_v<T>); instantiated for float
class ComplementaryFilter:
    T     alpha        # blend weight in [0, 1] — trust in the integrated (fast) path
    T     Ts           # sample period
    T     angle        # fused estimate (state)
    bool  wrapAngle    # true ⇒ treat state as a heading in (-pi, pi]
```

## Interface

```
ComplementaryFilter(T alpha, T Ts, T initial = 0)
T    Update(T rate, T measuredAngle)   # hot: rate = gyro (fast), measuredAngle = accel/mag (slow)
void Reset(T angle = 0)
void SetAlpha(T alpha)
static T AlphaFromTau(T tau, T Ts)     # tau = crossover time constant
```

## Algorithm (pseudocode)

```
function Update(rate, measuredAngle):        # OPTIMIZE_FOR_SPEED
    predicted = angle + rate * Ts            # high-pass path: integrate the fast sensor
    if wrapAngle:
        # blend along the shortest arc so the fusion is continuous across ±pi
        delta = WrapToPi(measuredAngle - predicted)
        angle = WrapToPi(predicted + (1 - alpha) * delta)
    else:
        # y = alpha*(angle + rate*Ts) + (1 - alpha)*measuredAngle
        angle = alpha * predicted + (1 - alpha) * measuredAngle
    return angle

function AlphaFromTau(tau, Ts):
    # first-order crossover: high-pass on gyro, low-pass on accel share cutoff 1/tau
    return tau / (tau + Ts)
```

## Complexity & memory

- Time: `O(1)` per sample — 2 multiplies, 2 adds (plus one wrap when tracking heading).
- Memory: `O(1)` — a single fused-angle state word and the coefficients.

## Numerical / embedded notes

- `alpha` near 1 (e.g. `0.98`) trusts the gyro over the short term while the accel/mag slowly
  corrects long-term **drift**; the two paths are exact frequency-domain complements: `HP + LP = 1`.
- Design via `alpha = tau/(tau+Ts)`, where `tau` is the high-pass/low-pass crossover time constant.
- **Angle wrap**: for heading fusion, blend the *shortest* angular distance (`WrapToPi`) — a naive
  linear blend jumps by `2*pi` near the ±pi seam.
- Unlike Mahony's filter it does not estimate gyro bias; pre-subtract a calibrated bias if needed.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/active/ComplementaryFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Update`, and
  `extern template class ComplementaryFilter<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/active/ComplementaryFilter.cpp` →
  `template class ComplementaryFilter<float>;`
- Test: `numerical/filters/active/test/TestComplementaryFilter.cpp`
- Doc: `doc/filters/active/ComplementaryFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestComplementaryFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
