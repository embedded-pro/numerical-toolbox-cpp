# Runge-Kutta ODE Integrators (RK4 + Dormand-Prince) — Implementation Pseudocode

> Roadmap ref: #24 (Tier 3) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t InputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
interface OdeSystem:
    Vector<T,StateSize> Derivative(x, u, t)          # ẋ = f(x,u,t)

template<typename T, std::size_t StateSize, std::size_t InputSize>
class RungeKutta4:                                   # fixed-step, deterministic
    OdeSystem<T,StateSize,InputSize>& system
    T          h                                     # fixed step

template<typename T, std::size_t StateSize, std::size_t InputSize>
class DormandPrince45:                               # embedded RK45, adaptive
    OdeSystem<T,StateSize,InputSize>& system
    T absTol, relTol
    T hMin, hMax
    Vector<T,StateSize> lastStage                    # FSAL: reuse k7 as next k1
```

## Interface

```
RungeKutta4(OdeSystem& system, T h)
Vector Step(x, u, t)                          # advance one fixed step; hot path

DormandPrince45(OdeSystem& system, T absTol, T relTol)
struct StepResult { Vector xNext; T hUsed; T hNext; bool accepted; }
StepResult Step(x, u, t, hSuggested)          # adaptive step; hot path
void SetStepBounds(T hMin, T hMax)
```

## Algorithm (pseudocode)

```
function Step_RK4(x, u, t):                    # OPTIMIZE_FOR_SPEED
    k1 = f(x,            u, t)
    k2 = f(x + h/2 · k1, u, t + h/2)
    k3 = f(x + h/2 · k2, u, t + h/2)
    k4 = f(x + h   · k3, u, t + h)
    return x + (h/6)·(k1 + 2·k2 + 2·k3 + k4)

function Step_DP45(x, u, t, h):                # OPTIMIZE_FOR_SPEED
    compute k1..k7 from the Dormand-Prince Butcher tableau   # k1 = FSAL from previous accept
    y5  = x + h · Σ b_i  · k_i                 # 5th-order solution
    y4  = x + h · Σ b*_i · k_i                 # 4th-order embedded solution
    err = weighted_norm(y5 − y4, absTol, relTol·|x|)
    if err <= 1:  accepted = true;  xNext = y5;  lastStage = k7
    else:         accepted = false; xNext = x
    hNew = h · clamp(0.9 · err^(−1/5), 0.2, 5.0)             # step-size control law
    return { xNext, h, clamp(hNew, hMin, hMax), accepted }
```

## Complexity & memory

- RK4: **4** RHS evaluations per step; `DP45`: **7** (**6** with FSAL reuse) per accepted step.
- Time per step `O(StageCount · StateSize)` plus the cost of `f`.
- Memory: `O(StateSize)` for the stage vectors — all stack/static, no heap.

## Numerical / embedded notes

- **Fixed-step RK4** is fully deterministic (constant work per tick) — the right choice for
  hard-real-time control loops; **DP45** trades determinism for accuracy offline or in HIL.
- **FSAL** (First Same As Last): DP45's 7th stage equals the next step's 1st, so an accepted step
  costs only 6 evaluations — cache it in `lastStage`.
- Clamp step growth/shrink (`0.2×…5×`) and honour `hMin`/`hMax` to keep step control stable.
- Explicit RK is **non-stiff**; document the stability limit `h·|λ| ≲ 2.8` for RK4 — a stiff plant
  needs a small fixed step or an implicit method.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/RungeKuttaIntegrators.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Step`, and
  `extern template class RungeKutta4<float, 1, 0>;` / `extern template class DormandPrince45<float, 1, 0>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/RungeKuttaIntegrators.cpp` →
  `template class RungeKutta4<float, 1, 0>;` and `template class DormandPrince45<float, 1, 0>;`
- Test: `numerical/solvers/test/TestRungeKuttaIntegrators.cpp`
- Doc: `doc/solvers/RungeKuttaIntegrators.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestRungeKuttaIntegrators.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
