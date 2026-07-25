# Product of Exponentials (Screw-Theory FK) — Implementation Pseudocode

> Roadmap ref: #M15 (Tier 3) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumLinks>      # static_assert(std::is_floating_point_v<T>); instantiated for float
class ProductOfExponentials:
    std::array<Vector<T,6>, NumLinks>  screws   # space-frame screw axes Sᵢ = (ωᵢ; vᵢ)
    SE3<T>                             home      # M: tool pose at q = 0
    bool bodyForm = false                        # space vs body PoE
```

## Interface

```
ProductOfExponentials(std::array<Vector<T,6>, N> screws, SE3<T> home)
SE3<T>           Compute(JointVector q)                     # ⁰Tₙ; hot path
Matrix<T, 6, N>  SpaceJacobian(JointVector q)               # columns = Adⱼ·Sᵢ
```

## Algorithm (pseudocode)

```
function Compute(q):                            # OPTIMIZE_FOR_SPEED  (space form)
    T = Identity
    for i in 0..N-1:
        T = T * SE3::Exp(screws[i], q[i])       # e^{[Sᵢ]qᵢ}  (reuse M6 / item 29)
    return T * home                             # T = e^{[S₁]q₁}···e^{[Sₙ]qₙ}·M

function SpaceJacobian(q):
    Js[0] = screws[0]
    T     = Identity
    for i in 1..N-1:
        T     = T * SE3::Exp(screws[i-1], q[i-1])
        Js[i] = Adjoint(T) * screws[i]           # transform screw into the current frame
    return Js
```

## Complexity & memory

- `Compute`: `O(N)` — one `SE(3)` exponential and one compose per joint.
- `SpaceJacobian`: `O(N)` — one adjoint (6×6) per column, reusing the running product.
- Memory: the `N` screw axes plus `M` and one accumulator; `O(1)` working set, no heap.

## Numerical / embedded notes

- PoE needs **no per-link frames** — every screw axis is expressed once in the fixed base frame, so
  there is no DH bookkeeping and no accumulated convention error (its main advantage over M7).
- Each `SE3::Exp` uses the closed-form `se(3)` exponential (Rodrigues + left Jacobian); guard the
  `θ → 0` case with a series expansion to avoid `sinθ/θ` division (reuse item 29 / M6).
- **Space vs body** form differ only in factor order and where `M` sits — expose the flag; space
  Jacobian columns are `Adⱼ·Sᵢ`, body Jacobian columns are `Adⱼ⁻¹·Bᵢ`.
- Normalize each `ωᵢ` to unit length (or zero for a prismatic screw) so `qᵢ` is a true angle/length.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/ProductOfExponentials.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ProductOfExponentials<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/ProductOfExponentials.cpp` → `template class ProductOfExponentials<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestProductOfExponentials.cpp`
- Doc: `doc/kinematics/ProductOfExponentials.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestProductOfExponentials.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
