# Model Reference Adaptive Control (MRAC) — Implementation Pseudocode

> Roadmap ref: #47 (Tier 5) · Target: `numerical/nonlinear_control` · Namespace `nonlinear_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
enum class AdaptationLaw { MitRule, Lyapunov }

template<typename T, std::size_t StateSize, std::size_t InputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ModelReferenceAdaptiveControl:
    const math::LinearTimeInvariant<T, StateSize, InputSize, StateSize>& reference  # A_m, B_m
    math::Vector<T, StateSize>              xm         # reference-model state
    math::Matrix<T, InputSize, StateSize>  thetaX      # adapted feedback params
    math::Matrix<T, InputSize, InputSize>  thetaR      # adapted feedforward params
    T              gamma                               # adaptation rate  γ > 0
    T              signB                               # sign of plant input gain (±1)
    AdaptationLaw  law
```

## Interface

```
# Reference model injected (DIP) — it defines the *desired* closed-loop behaviour:
ModelReferenceAdaptiveControl(const LinearTimeInvariant& referenceModel,
                              T gamma, T signB, AdaptationLaw law)

InputVector ComputeControl(const StateVector& x, const InputVector& r, T dt)      # hot path
void Reset()
```

## Algorithm (pseudocode)

```
function ComputeControl(x, r, dt):                    # OPTIMIZE_FOR_SPEED
    # 1. advance the reference model — the trajectory we WANT the plant to follow
    xm = xm + (reference.A * xm + reference.B * r) * dt

    # 2. tracking error between real plant and reference model
    e = x - xm

    # 3. adaptive control law:  u = θ_x·x + θ_r·r   (matches plant to reference when converged)
    u = thetaX * x + thetaR * r

    # 4. update parameters so e -> 0 (θ̇ = -γ·signB·e·regressorᵀ)
    #    MIT rule and Lyapunov redesign share this gradient form here;
    #    Lyapunov additionally guarantees boundedness of θ.
    thetaX = thetaX - (gamma * signB) * outer(e, x) * dt
    thetaR = thetaR - (gamma * signB) * outer(e, r) * dt

    return u

function Reset():
    xm = 0;  thetaX = 0;  thetaR = 0
```

## Complexity & memory

- `ComputeControl`: `O(StateSize²)` — reference-model step, two outer products, two matvecs.
- Memory: `O(StateSize² + StateSize·InputSize)` for the parameter matrices; static, no heap.

## Numerical / embedded notes

- **γ (adaptation gain)** trades tracking speed against stability: too large destabilizes,
  especially the pure **MIT rule**, which has *no* global stability proof. The **Lyapunov** law is
  derived to keep the error system's Lyapunov function non-increasing.
- **Parameter drift:** with noise or unmodelled disturbance, `θ` can wander even while `e` stays
  small. Add **σ-modification / e-modification** or **parameter projection** to bound `θ`.
- **Persistent excitation:** the command `r` must be rich enough for `θ` to converge to the true
  values; without it the plant still *tracks* but the parameters are not identified.
- `signB` must match the sign of the plant's high-frequency gain, or adaptation runs the wrong way.
- Reuse `math::LinearTimeInvariant` for the reference model; for an *indirect* adaptive variant,
  estimate the plant with `RecursiveLeastSquares` (`estimators/online`) and recompute the gains.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/nonlinear_control/ModelReferenceAdaptiveControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeControl`, and
  `extern template class ModelReferenceAdaptiveControl<float, StateSize, InputSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/nonlinear_control/ModelReferenceAdaptiveControl.cpp` →
  `template class ModelReferenceAdaptiveControl<float, StateSize, InputSize>;`
- Test: `numerical/nonlinear_control/test/TestModelReferenceAdaptiveControl.cpp`
- Doc: `doc/nonlinear_control/ModelReferenceAdaptiveControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestModelReferenceAdaptiveControl.cpp` → the `_test` target.
- New module: create `numerical/nonlinear_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/nonlinear_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
