# LMS / NLMS Adaptive Filter — Implementation Pseudocode

> Roadmap ref: #21 (Tier 3) · Target: `numerical/estimators/online` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Taps>         # static_assert(std::is_floating_point_v<T>); instantiated for float
class LmsAdaptiveFilter:
    Vector<T, Taps>          weights = 0      # adaptive FIR coefficients w
    RecursiveBuffer<T, Taps> x                # tapped delay line (newest .. oldest)
    T     mu                                   # step size (learning rate)
    bool  normalized                           # NLMS when true
    T     epsilon                              # NLMS regularizer (avoid /0)
```

## Interface

```
LmsAdaptiveFilter(T mu, bool normalized = false, T epsilon = small)
struct Result { T output; T error; }
Result Update(T input, T desired)            # hot path
const Vector<T, Taps>& Weights()
void Reset()
```

## Algorithm (pseudocode)

```
function Update(input, desired):             # OPTIMIZE_FOR_SPEED
    x.Push(input)                            # shift newest sample in
    y = dot(weights, x)                      # FIR output  y = wᵀx
    e = desired - y                          # estimation error
    step = mu
    if normalized:                            # NLMS
        step = mu / (epsilon + dot(x, x))     # divide by ‖x‖²
    # Steepest-descent weight update  w ← w + step·e·x
    for i in 0 .. Taps-1:
        weights[i] += step * e * x[i]
    return { y, e }

function Reset():
    weights = 0 ;  x.Clear()
```

## Complexity & memory

- Update: `O(Taps)` — two dot products and one AXPY, all length `Taps`.
- Memory: `O(Taps)` for weights plus the delay line; no heap, no covariance matrix (unlike RLS).

## Numerical / embedded notes

- **Stability bound:** plain LMS converges for `0 < mu < 2/λ_max(R)`; a safe rule of thumb is
  `mu < 2/(Taps·signalPower)`. NLMS removes the input-power dependence ⇒ use `0 < mu < 2`.
- NLMS `epsilon` prevents divide-by-zero and damps updates during silence (`‖x‖² ≈ 0`).
- Leaky-LMS variant `w ← (1−ρ)·w + step·e·x` bounds weight drift under non-persistent excitation.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/estimators/online/LmsAdaptiveFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Update`, and
  `extern template class LmsAdaptiveFilter<float, 4>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/online/LmsAdaptiveFilter.cpp` → `template class LmsAdaptiveFilter<float, 4>;`
- Test: `numerical/estimators/online/test/TestLmsAdaptiveFilter.cpp`
- Doc: `doc/estimators/LmsAdaptiveFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestLmsAdaptiveFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
