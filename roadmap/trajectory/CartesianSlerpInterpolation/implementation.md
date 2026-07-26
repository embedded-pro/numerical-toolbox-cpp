# Cartesian Path + Orientation (SLERP) Interpolation — Implementation Pseudocode

> Roadmap ref: #M10 (Tier 2) · Target: `numerical/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct Pose:                       # reuses existing types
    math::Vector<T, 3>  position   # Cartesian point (m)
    math::Quaternion<T> orientation  # unit quaternion (item #18)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class CartesianSlerpInterpolation:
    Pose<T>  start, goal
    T        duration              # tf > 0
    T        dot, theta, sinTheta  # precomputed SLERP geometry
    bool     useNlerp              # near-parallel fallback flag
```

## Interface

```
CartesianSlerpInterpolation(Pose<T> start, Pose<T> goal, T tf)
Pose<T> Sample(T t)                 # hot path
T       Duration()
void    SetTimeScaling(scaling)     # linear, or a Trapezoidal/Polynomial s(t) driver
```

## Algorithm (pseudocode)

```
function plan():                              # precompute orientation geometry once
    if dot(start.q, goal.q) < 0:              # pick the shortest of the two arcs
        goal.q = -goal.q
    dot = clamp(dot(start.q, goal.q), -1, 1)
    theta = acos(dot)
    useNlerp = (dot > 0.9995)                 # arc too small ⇒ linear blend + normalize
    sinTheta = sin(theta)

function Sample(t):                           # OPTIMIZE_FOR_SPEED
    s = timeScaling(clamp(t, 0, duration) / duration)   # s ∈ [0, 1]
    # position: straight line in Cartesian space
    p = start.position + s * (goal.position - start.position)
    # orientation: spherical linear interpolation
    if useNlerp:
        q = normalize( (1 - s)*start.q + s*goal.q )
    else:
        w0 = sin((1 - s)*theta) / sinTheta
        w1 = sin(     s *theta) / sinTheta
        q = w0*start.q + w1*goal.q            # already unit-length
    return Pose{ p, q }
```

## Complexity & memory

- Planning: `O(1)` — one `dot`, one `acos`, one `sin`.
- `Sample`: `O(1)` — a 3-vector lerp plus two `sin` (or an nlerp + one normalize).
- Memory: `O(1)` — two poses and a few cached scalars; stack-resident, no heap.

## Numerical / embedded notes

- Reuse `math::Quaternion::Slerp` / `Normalize` (item #18) rather than re-deriving the blend.
- **Shortest-path fix:** negate `goal.q` when `dot < 0`; antipodal quaternions are the same rotation
  but the long way round.
- **`nlerp` fallback** when `dot → 1` avoids the `1/sin θ` singularity for near-parallel orientations.
- Decouple position and orientation timing by sharing one scalar `s(t)` from a
  `TrapezoidalProfile`/`PolynomialTrajectory`, so both reach the goal simultaneously.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/trajectory/CartesianSlerpInterpolation.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class CartesianSlerpInterpolation<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/trajectory/CartesianSlerpInterpolation.cpp` →
  `template class CartesianSlerpInterpolation<float>;`
- Test: `numerical/trajectory/test/TestCartesianSlerpInterpolation.cpp`
- Doc: `doc/trajectory/CartesianSlerpInterpolation.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestCartesianSlerpInterpolation.cpp` → the `_test` target.
- New module: create `numerical/trajectory/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/CMakeLists.txt`, and add a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
