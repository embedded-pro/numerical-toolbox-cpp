# Analytical IK — Pieper (Wrist-Partitioned 6R) — Implementation Pseudocode

> Roadmap ref: #M21 (Tier 4) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct IkSolutions:
    std::array<Vector<T,6>, 8>  q                # up to 8 branches
    std::size_t                 count            # how many are valid / reachable

template<typename T>
class AnalyticalIkPieper:                        # fixed 6R with a spherical wrist
    std::array<DhLink<T>, 6>  dh                 # last three axes intersect at the wrist
```

## Interface

```
AnalyticalIkPieper(std::array<DhLink<T>, 6> dh)
IkSolutions<T>   Solve(SE3<T> target)            # all real solutions; hot path
```

## Algorithm (pseudocode)

```
function Solve(target):                          # OPTIMIZE_FOR_SPEED  (closed form, no iteration)
    # 1. wrist centre — subtract the tool offset along the approach axis
    p_wc = target.p - d6 * (target.R * ẑ)

    # 2. arm (joints 1-3) from the wrist-centre position — planar geometry
    θ1 = atan2(p_wc.y, p_wc.x)              # and the θ1 + π branch
    (r, s) = planar coords of p_wc in the θ1 plane
    c3 = (r² + s² - a2² - a3²) / (2·a2·a3)
    if |c3| > 1: this branch is unreachable — skip it
    θ3 = ±acos(c3)                          # elbow-up / elbow-down
    θ2 = atan2(s, r) - atan2(a3·sinθ3, a2 + a3·cosθ3)

    # 3. wrist (joints 4-6) from the leftover orientation
    R03 = rotation of joints 1-3
    R36 = Transpose(R03) * target.R
    (θ4, θ5, θ6) = ZYZ-Euler(R36)           # two branches: θ5 = ±acos(R36[2,2])

    # 4. assemble the (≤ 2×2×2 = 8) combinations into IkSolutions
    return solutions
```

## Complexity & memory

- `O(1)` — a fixed number of `atan2` / `acos` calls; **no iteration, no Jacobian, no matrix solve**.
- Enumerates up to `8` postures (2 shoulder × 2 elbow × 2 wrist).
- Memory: the 8-solution array; entirely on the stack.

## Numerical / embedded notes

- **Reachability filter:** if the law-of-cosines argument `|c3| > 1` the target is out of reach on that
  branch — drop it instead of feeding `acos` an out-of-domain value.
- **Wrist singularity** (`θ5 → 0`, axes 4 and 6 align): `θ4` and `θ6` are individually undefined, only
  their sum is fixed — pick a convention (e.g. hold `θ4` at its current value) and flag it.
- Requires a **spherical wrist** (last three axes intersect); the partition into position (1-3) and
  orientation (4-6) is what makes the closed form possible — reuse M6/M7 for the transforms.
- Enumerate *all* real solutions and let the caller choose (nearest to current `q`, joint-limit-feasible).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/AnalyticalIkPieper.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class AnalyticalIkPieper<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/AnalyticalIkPieper.cpp` → `template class AnalyticalIkPieper<float>;`
- Test: `numerical/kinematics/test/TestAnalyticalIkPieper.cpp`
- Doc: `doc/kinematics/AnalyticalIkPieper.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestAnalyticalIkPieper.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
