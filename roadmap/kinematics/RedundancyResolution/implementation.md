# Redundancy Resolution (Null-Space Projection) — Implementation Pseudocode

> Roadmap ref: #M14 (Tier 3) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumLinks>      # static_assert(std::is_floating_point_v<T>); instantiated for float
class RedundancyResolution:                     # NumLinks > 6 (redundant)
    SpatialJacobian<T, NumLinks>  jac
    T  damping                                  # λ for the damped pseudo-inverse
    # scratch: JJt (6×6), Jpinv (N×6), P (N×N)
```

## Interface

```
RedundancyResolution(SpatialJacobian<T, NumLinks> jac, T damping = 0)
Matrix<T, N, 6>  PseudoInverse(JointVector q)                     # J⁺
Matrix<T, N, N>  NullSpaceProjector(JointVector q)                # I − J⁺J
JointVector      Resolve(JointVector q, Vector<T,6> xdot,
                         JointVector qdot0)                       # hot path
```

## Algorithm (pseudocode)

```
function PseudoInverse(q):                       # right inverse, wide J
    J   = jac.Compute(q)                         # 6×N
    JJt = J * Transpose(J) + λ² I₆               # damped ⇒ singularity-robust
    #  J⁺ = Jᵀ (JJt)⁻¹  via 6×6 solves (reuse GaussianElimination / QR item 27)
    return Transpose(J) * Inverse(JJt)

function NullSpaceProjector(q):
    J  = jac.Compute(q)
    Jp = PseudoInverse(q)
    return I_N - Jp * J                           # N×N, idempotent

function Resolve(q, xdot, qdot0):                # OPTIMIZE_FOR_SPEED
    Jp        = PseudoInverse(q)
    qdot_task = Jp * xdot                         # minimum-norm primary solution
    P         = I_N - Jp * jac.Compute(q)         # null-space projector
    return qdot_task + P * qdot0                  # secondary objective in the null space
```

## Complexity & memory

- `PseudoInverse`: `O(6²N + 6³)` — form `J Jᵀ`, one 6×6 factorization, back-substitute.
- `Resolve`: adds an `O(N²)` projector multiply for the secondary term.
- Memory: `N×6`, `N×N`, and `6×6` scratch; all bounded, stack-allocated.

## Numerical / embedded notes

- The **right** pseudo-inverse `Jᵀ(J Jᵀ)⁻¹` is correct for a *wide* (redundant) `J`; the left form
  applies to tall `J`. Damping `λ²` keeps `J Jᵀ` invertible near singularities.
- `P = I − J⁺J` is a **projector**: `P² = P`. Anything it multiplies moves the joints *without*
  disturbing the tool — so the secondary objective is exactly task-consistent.
- Common secondary rates `q̇₀`: gradient of manipulability (M11), distance-to-joint-limits, or an
  obstacle-avoidance potential. Scale `q̇₀` so it never dominates the primary task.
- For best conditioning build `J⁺` from a **QR/SVD** (items 27/43) rather than an explicit inverse.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/RedundancyResolution.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Resolve`, and
  `extern template class RedundancyResolution<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/RedundancyResolution.cpp` → `template class RedundancyResolution<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestRedundancyResolution.cpp`
- Doc: `doc/kinematics/RedundancyResolution.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestRedundancyResolution.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
