# Sliding Mode Control (SMC) — Implementation Pseudocode

> Roadmap ref: #34 (Tier 4) · Target: `numerical/robust_control` · Namespace `robust_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t InputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SlidingModeControl:
    math::LinearTimeInvariant<T, StateSize, InputSize, StateSize> plant  # A, B (C = I)
    math::Matrix<T, InputSize, StateSize>  surface     # S: sliding-surface gain, s = S·x
    math::Matrix<T, InputSize, InputSize>  SBinv       # (S·B)^{-1}, precomputed
    math::Vector<T, InputSize>             switchGain  # K per input channel
    T                                      boundaryLayer  # φ > 0
```

## Interface

```
SlidingModeControl(const LinearTimeInvariant& plant,
                   const Matrix<T,InputSize,StateSize>& surface,
                   const Vector<T,InputSize>& K,
                   T phi)

InputVector ComputeControl(const StateVector& x)                       # hot path
InputVector ComputeControl(const StateVector& x, const StateVector& reference)
Vector<T,InputSize> Surface(const StateVector& x) const               # s(x) for monitoring
void        SetBoundaryLayer(T phi)
static T    Sat(T s, T phi)              # boundary-layer saturation, replaces sign()
```

## Algorithm (pseudocode)

```
function ComputeControl(x):                     # OPTIMIZE_FOR_SPEED
    s     = surface * x                         # sliding variable s = S·x
    # equivalent control keeps the state on s = 0 (ṡ = 0 for the nominal plant):
    #   u_eq = -(S·B)^{-1} · S·A · x
    u_eq  = SBinv * (surface * (plant.A * x)) * (-1)
    # robust reaching term with boundary layer to suppress chattering:
    #   u_sw = (S·B)^{-1} · K · sat(s / φ)
    u_sw  = SBinv * ScaledSat(switchGain, s, boundaryLayer)
    return u_eq - u_sw

function Sat(s, phi):
    # linear ramp inside the boundary layer, ±1 outside — a continuous sign()
    if abs(s) <= phi: return s / phi
    else:             return sign(s)
```

## Complexity & memory

- `ComputeControl`: `O(StateSize²)` for the `S·A·x` product, plus `O(InputSize²)` for the `(S·B)^{-1}` applies.
- Design-time: one `O(InputSize³)` inversion of `S·B`.
- Memory: `O(StateSize·InputSize)` for the gains — all static, no heap.

## Numerical / embedded notes

- **Boundary layer `φ`** trades chattering against accuracy: `φ→0` approaches ideal sliding (infinite
  switching), larger `φ` smooths the control at the cost of a `O(φ)` steady-state boundary-layer error.
- `Sat()` replaces the discontinuous `sign()` so the control has finite bandwidth — pure switching
  excites unmodeled high-frequency dynamics and wears actuators.
- **Reaching condition:** `K` must dominate the matched-disturbance bound (`K > ‖matched d‖`) so
  `s·ṡ < 0` and the state provably reaches the surface in finite time.
- Requires `S·B` **nonsingular** (relative-degree-one surface); assert this precondition at construction.
- Reuse item 3 (`SaturationRateLimiter`) semantics for the boundary-layer clamp — do not reimplement.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/robust_control/SlidingModeControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeControl`, and
  `extern template class SlidingModeControl<float, StateSize, InputSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/robust_control/SlidingModeControl.cpp` →
  `template class SlidingModeControl<float, StateSize, InputSize>;`
- Test: `numerical/robust_control/test/TestSlidingModeControl.cpp`
- Doc: `doc/robust_control/SlidingModeControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSlidingModeControl.cpp` → the `_test` target.
- New module: create `numerical/robust_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/robust_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
