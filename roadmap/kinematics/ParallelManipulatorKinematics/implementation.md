# Parallel Manipulator Kinematics (Delta / Stewart-Gough) — Implementation Pseudocode

> Roadmap ref: #M23 (Tier 4) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumLegs>       # static_assert(std::is_floating_point_v<T>); instantiated for float
struct PlatformGeometry:
    std::array<Vector3<T>, NumLegs> baseAnchors     # bᵢ in the fixed frame
    std::array<Vector3<T>, NumLegs> platformAnchors # pᵢ in the moving frame

template<typename T>
struct ForwardConfig:
    T           tolerance
    std::size_t maxIterations

template<typename T, std::size_t NumLegs>
class ParallelManipulatorKinematics:
    PlatformGeometry<T, NumLegs>  geom
    ForwardConfig<T>              config
```

## Interface

```
ParallelManipulatorKinematics(geom, ForwardConfig<T> cfg = {})
std::array<T, NumLegs>  Inverse(SE3<T> platformPose)             # leg lengths; closed form, hot path
SE3<T>                  Forward(std::array<T, NumLegs> legLengths,
                                SE3<T> guess)                    # Newton iteration
```

## Algorithm (pseudocode)

```
function Inverse(pose):                          # OPTIMIZE_FOR_SPEED  (closed form)
    for i in 0..NumLegs-1:
        legVec    = pose.R * platformAnchors[i] + pose.p - baseAnchors[i]
        length[i] = VectorNorm(legVec)           # each leg independent
    return length

function Forward(measured, guess):               # iterative — multiple assembly modes
    x = guess                                    # pose as (p, small-rotation)
    for iter in 0..maxIterations-1:
        f = Inverse(x) - measured                # residual per leg
        if norm(f) < tolerance: return x
        J  = ∂(legLengths)/∂x                    # NumLegs × 6 (Stewart), analytic or numeric
        Δx = solve(J, -f)                        # LU (item 28); least squares if NumLegs ≠ 6
        x  = x ⊕ Δx                              # retract onto SE(3)
    return x                                     # best effort (may not converge)
```

## Complexity & memory

- `Inverse`: `O(NumLegs)` — one transform + norm per leg, fully parallel and branch-free.
- `Forward`: `O(iters · 6³)` — a `6×6` (or least-squares) solve per Newton step.
- Memory: geometry arrays plus one `NumLegs×6` Jacobian; bounded, stack-allocated.

## Numerical / embedded notes

- Parallel robots invert the usual difficulty: **inverse** kinematics is trivial and closed-form, while
  **forward** kinematics needs Newton iteration (the opposite of a serial arm).
- Forward FK has **multiple assembly modes** (the platform can sit in several poses for the same leg
  lengths) — the `guess` selects the branch; seed it from the previous cycle.
- The **Delta** (3 legs, parallelogram arms) admits a closed-form forward solve via three-sphere
  intersection — specialize it rather than iterating.
- Near a platform singularity the leg Jacobian is ill-conditioned — monitor it (M11 idea) and fall back
  to a damped step; reuse `GaussianElimination` / LU (item 28) for the linear solve.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/ParallelManipulatorKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Inverse`, and
  `extern template class ParallelManipulatorKinematics<float, NumLegs>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/ParallelManipulatorKinematics.cpp` → `template class ParallelManipulatorKinematics<float, NumLegs>;`
- Test: `numerical/kinematics/test/TestParallelManipulatorKinematics.cpp`
- Doc: `doc/kinematics/ParallelManipulatorKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestParallelManipulatorKinematics.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
