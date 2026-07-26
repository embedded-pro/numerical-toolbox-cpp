# Spatial Jacobian (6×N) — Implementation Pseudocode

> Roadmap ref: #M8 (Tier 2) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumLinks>     # static_assert(std::is_floating_point_v<T>); instantiated for float
class SpatialJacobian:
    # geometric Jacobian in the base frame:
    #   Matrix<T, 6, NumLinks>  columns = [ linear (3); angular (3) ]
    DenavitHartenberg<T, NumLinks>  model      # or a screw / SE(3) chain
```

## Interface

```
SpatialJacobian(DenavitHartenberg<T, NumLinks> model)
Matrix<T, 6, N>   Compute(JointVector q)                            # geometric J; hot path
Vector<T, 6>      EndEffectorTwist(JointVector q, JointVector qdot) # J·q̇
JointVector       JointTorques(JointVector q, Vector<T,6> wrench)   # Jᵀ·w  (statics)
```

## Algorithm (pseudocode)

```
function Compute(q):                            # OPTIMIZE_FOR_SPEED
    frames = model.FrameChain(q)                # every ⁰Tᵢ (reuse M7/M6)
    oₙ = frames[N].p                            # end-effector origin
    for i in 0..N-1:
        zᵢ = frames[i].R * ẑ                    # joint axis in base frame
        oᵢ = frames[i].p
        if joint i is Revolute:
            Jv = CrossProduct(zᵢ, oₙ - oᵢ)      # reuse Geometry3D
            Jw = zᵢ
        else:                                   # Prismatic
            Jv = zᵢ
            Jw = 0
        column i = concat(Jv, Jw)               # 6-vector
    return J

function EndEffectorTwist(q, qdot):
    return Compute(q) * qdot                    # (v; ω)

function JointTorques(q, w):                    # OPTIMIZE_FOR_SPEED
    return Transpose(Compute(q)) * w            # τ = Jᵀ w
```

## Complexity & memory

- `Compute`: `O(N)` — one `FrameChain` pass plus a cross product per column.
- `EndEffectorTwist` / `JointTorques`: `O(6N)` matrix-vector products.
- Memory: one `6×N` matrix plus the `N+1` cached frames; all stack-allocated.

## Numerical / embedded notes

- This **promotes** the private 3×N position Jacobian inside `InverseKinematics.hpp` to the full 6×N
  form — the single dependency that unblocks pose-IK, manipulability, and redundancy.
- Twist ordering is `(v; ω)` (linear first) here; downstream consumers must match this block layout.
- Near a **singularity** two columns become linearly dependent (rank drops) — detectable via the
  manipulability index (M11); do not invert `J` directly, use damped least squares.
- `Jᵀ` gives the statics/force map for free — the transpose needs no recomputation.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/SpatialJacobian.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`/`JointTorques`, and
  `extern template class SpatialJacobian<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/SpatialJacobian.cpp` → `template class SpatialJacobian<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestSpatialJacobian.cpp`
- Doc: `doc/kinematics/SpatialJacobian.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSpatialJacobian.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
