# Disturbance Observer (DOB) — Implementation Pseudocode

> Roadmap ref: #35 (Tier 4) · Target: `numerical/robust_control` · Namespace `robust_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class DisturbanceObserver:
    math::LinearTimeInvariant<T,StateSize,InputSize,OutputSize> nominalPlant  # Pn: A,B,C,D
    filters::passive::BiquadCascade<T, QOrder> qFilter  # Q(z): low-pass, unity DC gain
    NominalInverseState inverseState  # realizes Q·Pn^{-1}·y
    math::Vector<T,InputSize> disturbance  # d̂  (lumped estimate)
    math::Vector<T,InputSize> appliedPrev  # u applied last step (for Q·u)
```

## Interface

```
DisturbanceObserver(const LinearTimeInvariant& nominalPlant, const QFilterCoeffs& q)

InputVector Compute(const InputVector& nominalControl,        # hot path -> u = c - d̂
                    const OutputVector& measuredOutput)
const InputVector& Disturbance() const                        # expose d̂ for monitoring
void Reset()
```

## Algorithm (pseudocode)

```
function Compute(c, y):                          # OPTIMIZE_FOR_SPEED
    # 1. Drive measured output through the *proper* cascade Q(z)·Pn^{-1}(z)
    a = ApplyQtimesPinv(inverseState, y)         # Q·Pn^{-1}·y   (realizable: rel-deg Q ≥ Pn)
    # 2. Drive the previously-applied control through the same Q(z)
    b = qFilter.Filter(appliedPrev)              # Q·u
    # 3. Lumped disturbance estimate (model mismatch + external load)
    #    d̂ = Q·Pn^{-1}·y − Q·u
    disturbance = a - b
    # 4. Cancel it inside the loop
    u = c - disturbance
    appliedPrev = u
    return u
```

## Complexity & memory

- `Compute`: `O(StateSize²)` for the nominal-inverse realization, plus `O(QOrder)` for the biquad Q-filter.
- Memory: `O(StateSize²)` for `Pn`/its inverse-realization, `O(QOrder)` for `Q`, all static — no heap.

## Numerical / embedded notes

- **Q-filter bandwidth is the master knob:** inside `Q`'s passband disturbances are rejected and the
  actual plant is forced to behave like `Pn`; beyond it the DOB is transparent (`Q ≈ 0`).
- **Properness:** `Q` must have relative degree ≥ `Pn` so `Q·Pn^{-1}` is realizable — otherwise it
  differentiates and amplifies measurement noise.
- **Unity DC gain** on `Q` (`Q(1) = 1`) gives complete rejection of constant (DC) disturbances.
- **Robustness trade-off:** widening `Q` improves rejection but shrinks the robust-stability margin to
  model mismatch and amplifies sensor noise — tune against the plant's uncertainty.
- Reuse item 15 (`BiquadCascade`) for `Q` and item 19 (observer) machinery for the inverse; do not
  reimplement either (DRY).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/robust_control/DisturbanceObserver.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class DisturbanceObserver<float, StateSize, InputSize, OutputSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/robust_control/DisturbanceObserver.cpp` →
  `template class DisturbanceObserver<float, StateSize, InputSize, OutputSize>;`
- Test: `numerical/robust_control/test/TestDisturbanceObserver.cpp`
- Doc: `doc/robust_control/DisturbanceObserver.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestDisturbanceObserver.cpp` → the `_test` target.
- New module: create `numerical/robust_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/robust_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
