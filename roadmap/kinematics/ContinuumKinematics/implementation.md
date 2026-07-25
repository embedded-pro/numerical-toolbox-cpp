# Continuum Kinematics (Constant Curvature) — Implementation Pseudocode

> Roadmap ref: #M26 (Tier 4) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ArcParameters:                           # configuration space of one section
    T kappa      # curvature  (1 / radius)
    T phi        # bending-plane angle
    T length     # arc length s

template<typename T, std::size_t NumSections>
class ContinuumKinematics:
    std::array<ArcParameters<T>, NumSections>  sections
```

## Interface

```
ContinuumKinematics(std::array<ArcParameters<T>, NumSections> sections)
SE3<T>            SectionTransform(ArcParameters<T> arc)         # one arc; hot path
SE3<T>            Forward()                                       # tip pose; hot path
ArcParameters<T>  InverseSection(SE3<T> sectionPose)             # single-section geometric IK
```

## Algorithm (pseudocode)

```
function SectionTransform(arc):                 # OPTIMIZE_FOR_SPEED  (robot-independent map)
    θ = arc.kappa * arc.length                  # total bend angle
    if arc.kappa ≈ 0:                            # straight section — series fallback
        return SE3(I, (0, 0, arc.length))
    # arc lies in a plane rotated by φ about z; circle of radius 1/κ
    p = (1/κ) * ( (1 - cosθ)·(cosφ, sinφ, 0) + sinθ·ẑ )
    R = Rotz(φ) * Roty(θ) * Rotz(-φ)             # frame swept along the arc
    return SE3(R, p)

function Forward():                             # OPTIMIZE_FOR_SPEED
    T = Identity
    for i in 0..NumSections-1:
        T = T * SectionTransform(sections[i])   # reuse SE(3) compose (M6)
    return T

function InverseSection(pose):                  # single section, closed form
    # recover (κ, φ, s) from one constant-curvature arc's tip
    φ = atan2(pose.p.y, pose.p.x)
    θ = 2 * atan2( ‖(pose.p.x, pose.p.y)‖ , pose.p.z )   # from arc geometry
    κ = θ / arcLengthFrom(pose.p, θ)
    s = θ / κ
    return { κ, φ, s }
```

## Complexity & memory

- `SectionTransform`: `O(1)` — a few trig calls and one `SE(3)` build.
- `Forward`: `O(NumSections)` `SE(3)` products; `InverseSection`: `O(1)` closed form.
- Memory: the section array plus one accumulator; no heap.

## Numerical / embedded notes

- **Curvature `κ → 0` is the trap:** the pose uses `1/κ`, which blows up for a straight section — switch
  to the `θ → 0` series (`p → (0,0,s)`) below a threshold, and note `φ` is undefined when straight.
- Keep the **robot-independent** map `(κ, φ, s) → SE(3)` separate from the **robot-specific** map
  (tendon lengths / chamber pressures → `(κ, φ, s)`); only the latter changes between hardware.
- Constant-curvature is a **modeling assumption** (piecewise circular arcs); real gravity/load bending
  deviates from it — document it as an approximation, not exact kinematics.
- Multi-section inverse kinematics generally needs iteration (reuse damped least squares, M13-style);
  only the single-section case is closed-form here.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/ContinuumKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `SectionTransform`/`Forward`, and
  `extern template class ContinuumKinematics<float, NumSections>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/ContinuumKinematics.cpp` → `template class ContinuumKinematics<float, NumSections>;`
- Test: `numerical/kinematics/test/TestContinuumKinematics.cpp`
- Doc: `doc/kinematics/ContinuumKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestContinuumKinematics.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
