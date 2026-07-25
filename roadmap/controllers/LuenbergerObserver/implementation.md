# Luenberger Observer (Pole Placement / Ackermann) — Implementation Pseudocode

> Roadmap ref: #19 (Tier 3) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class LuenbergerObserver:
    math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize> plant   # A,B,C,D
    math::Matrix<T, StateSize, OutputSize> L        # observer gain
    math::Vector<T, StateSize>             xhat     # state estimate
```

## Interface

```
# Gain designed offline and injected:
LuenbergerObserver(const LinearTimeInvariant& plant,
                   const Matrix<T,StateSize,OutputSize>& L)

# Or designed in-place from desired observer poles (SISO output):
static Matrix<T,StateSize,1> AckermannGain(
        const LinearTimeInvariant& plant,
        const std::array<T,StateSize>& desiredPoles)

Vector<T,StateSize> Update(const InputVector& u, const OutputVector& y)   # hot path
const Vector<T,StateSize>& Estimate() const
void Reset(const Vector<T,StateSize>& x0)
```

## Algorithm (pseudocode)

```
function Update(u, y):                          # OPTIMIZE_FOR_SPEED
    yhat        = plant.C * xhat + plant.D * u          # predicted output
    innovation  = y - yhat                              # measurement residual
    # x̂[k+1] = A x̂ + B u + L (y - C x̂)
    xhat = plant.A * xhat + plant.B * u + L * innovation
    return xhat

function AckermannGain(plant, desiredPoles):
    # Dual of controller pole placement (observability form)
    O   = ObservabilityMatrix(plant.A, plant.C)         # [C; CA; ...; CA^(n-1)]
    phi = EvaluateCharacteristicPoly(plant.A, desiredPoles)   # φ_d(A)
    # L = φ_d(A) · O^{-1} · e_last
    return phi * Inverse(O) * UnitVectorLast(StateSize)
```

## Complexity & memory

- `Update`: `O(StateSize²)` for the `A·x̂` and `L·innovation` products (dense matrices).
- `AckermannGain` (design-time): `O(StateSize³)` for the observability-matrix solve.
- Memory: `O(StateSize²)` for `A`/`L`, `O(StateSize)` for the estimate — all static.

## Numerical / embedded notes

- Place observer poles **faster** than the controller poles (≈2–5×) so estimation error decays
  before it corrupts feedback — but not so fast that measurement noise is amplified.
- Requires an **observable** `(A, C)` pair; check with the observability rank test (item 26) before
  trusting Ackermann's formula (which assumes full rank).
- Reuse `GaussianElimination` for the `O^{-1}` solve rather than forming an explicit inverse.
- The estimate feeds a `StateFeedbackController` (e.g. `Lqr`); together they form an
  observer-based compensator (the deterministic sibling of LQG).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/LuenbergerObserver.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Update`, and
  `extern template class LuenbergerObserver<float, StateSize, InputSize, OutputSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/LuenbergerObserver.cpp` →
  `template class LuenbergerObserver<float, StateSize, InputSize, OutputSize>;`
- Test: `numerical/controllers/implementations/test/TestLuenbergerObserver.cpp`
- Doc: `doc/controllers/LuenbergerObserver.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestLuenbergerObserver.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
