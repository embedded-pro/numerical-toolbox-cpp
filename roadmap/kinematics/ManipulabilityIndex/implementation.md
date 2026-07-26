# Manipulability Index (Yoshikawa) — Implementation Pseudocode

> Roadmap ref: #M11 (Tier 2) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumLinks>     # static_assert(std::is_floating_point_v<T>); instantiated for float
class ManipulabilityIndex:
    SpatialJacobian<T, NumLinks>  jac
    # scratch: SquareMatrix<T, 6>  JJt  (task-space Gram, m = 6)
```

## Interface

```
ManipulabilityIndex(SpatialJacobian<T, NumLinks> jac)
T                 Compute(JointVector q)                 # w = √det(J Jᵀ); hot path
T                 ConditionNumber(JointVector q)         # σ_max / σ_min
std::array<T, 6>  EllipsoidAxes(JointVector q)           # singular values (semi-axis lengths)
bool              NearSingular(JointVector q, T eps)      # w < eps
```

## Algorithm (pseudocode)

```
function Compute(q):                            # OPTIMIZE_FOR_SPEED
    J   = jac.Compute(q)                        # 6×N
    JJt = J * Transpose(J)                       # 6×6, symmetric PSD
    return sqrt( determinant(JJt) )             # Yoshikawa measure w
    # square non-redundant arm (N = 6): w = |det J| directly — skip the product

function EllipsoidAxes(q):
    J = jac.Compute(q)
    σ = SingularValues(J)                        # reuse SVD (item 43)
    return σ                                     # w = ∏ σᵢ ; axes of the velocity ellipsoid

function ConditionNumber(q):
    σ = SingularValues(jac.Compute(q))
    return σ_max / σ_min                          # → ∞ at a singularity

function NearSingular(q, eps):
    return Compute(q) < eps
```

## Complexity & memory

- `Compute` via `J Jᵀ` + determinant: `O(N·36 + 6³)` — cheap for small `N`.
- `EllipsoidAxes` / `ConditionNumber` via SVD: `O(6²·N)` but more numerically robust.
- Memory: one `6×6` scratch matrix (or the SVD factors); no heap.

## Numerical / embedded notes

- `w = √det(J Jᵀ)` is the **volume** of the velocity ellipsoid: large = dexterous, `w → 0` = singular.
- Prefer **SVD** (item 43) over an explicit determinant when you also need conditioning or the
  ellipsoid axes — `det(J Jᵀ) = (∏ σᵢ)²`, but the product of singular values avoids cancellation.
- For a **redundant** arm (`N > 6`) the Gram-determinant root is the only valid form — `det J` does
  not exist for a non-square `J`.
- Use `NearSingular` as a guard *before* any Jacobian inverse; do not wait for the solve to blow up.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/ManipulabilityIndex.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ManipulabilityIndex<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/ManipulabilityIndex.cpp` → `template class ManipulabilityIndex<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestManipulabilityIndex.cpp`
- Doc: `doc/kinematics/ManipulabilityIndex.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestManipulabilityIndex.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
