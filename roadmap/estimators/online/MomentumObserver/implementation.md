# Momentum-Based Collision-Detection Observer — Implementation Pseudocode

> Roadmap ref: #M16 (Tier 3) · Target: `numerical/estimators/online` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class MomentumObserver:
    Dynamics<T, Dof>&  model                  # supplies M(q), Cᵀ(q,q̇)q̇, g(q)
    Vector<T, Dof>     gain                    # observer bandwidth K_O (diagonal, > 0)
    Vector<T, Dof>     integral = 0            # ∫(τ + Cᵀq̇ − g + r) dt
    Vector<T, Dof>     residual = 0            # r ≈ estimated external torque
    Vector<T, Dof>     p0                       # generalized momentum captured at reset
```

## Interface

```
MomentumObserver(Dynamics<T, Dof>& model, Vector<Dof> gain)
Vector<Dof> Update(q, qd, tau, dt)           # hot path; returns residual r
void Reset(q, qd)                             # capture p0 = M(q)·q̇, clear integral
bool CollisionDetected(threshold)             # |r_i| > threshold_i on any joint
```

## Algorithm (pseudocode)

```
function Update(q, qd, tau, dt):             # OPTIMIZE_FOR_SPEED
    p = model.MassMatrix(q) * qd             # generalized momentum  p = M(q) q̇
    # β(q,q̇) = Cᵀ(q,q̇) q̇ − g(q)   (uses skew-symmetry  Ṁ = C + Cᵀ)
    beta = model.CoriolisTranspose(q, qd) - model.Gravity(q)
    integral += (tau + beta + residual) * dt  # rectangular integration
    residual = elementwise(gain, p - p0 - integral)   # r = K_O ⊙ (p − p0 − ∫)
    return residual

function Reset(q, qd):
    p0 = model.MassMatrix(q) * qd
    integral = 0 ;  residual = 0
```

## Complexity & memory

- Update: one `M(q)` / `Cᵀq̇` / `g(q)` evaluation (`O(Dof²)`–`O(Dof)` via RNEA) plus `O(Dof)` glue.
- Memory: a few `Dof`-vectors; no heap, no matrix inversion, no acceleration term.

## Numerical / embedded notes

- **No q̈, no torque sensors:** the observer avoids noisy acceleration estimates entirely; each
  residual channel `r_i` behaves like a **first-order low-pass of the true external torque τ_ext**
  with bandwidth set by `gain_i` — larger gain = faster but noisier, smaller = smoother but laggier.
- Decoupling: with diagonal `K_O` the residual dynamics decouple per joint ⇒ a collision on link
  `j` shows up predominantly in `r_j` (and downstream joints), aiding localization.
- Integrate with a fixed `dt`; model error (gravity/friction bias) drifts the `integral`, so
  bias-compensate or periodically re-`Reset()` when the arm is known to be contact-free.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/estimators/online/MomentumObserver.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Update`, and
  `extern template class MomentumObserver<float, 2>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/online/MomentumObserver.cpp` → `template class MomentumObserver<float, 2>;`
- Test: `numerical/estimators/online/test/TestMomentumObserver.cpp`
- Doc: `doc/estimators/MomentumObserver.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestMomentumObserver.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
