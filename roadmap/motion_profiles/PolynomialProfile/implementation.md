# Polynomial Point-to-Point Profile — Implementation Pseudocode

> Roadmap ref: #55 (Tier 1) · Target: `numerical/motion_profiles` · Namespace `motion_profiles` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Shared motion-profile types — **canonical definition**, created once in
`numerical/motion_profiles/MotionProfileTypes.hpp` by the first `motion_profiles` item deployed; every other
profile spec includes it and never redefines these names (robotics-toolbox-cpp's time laws consume them too):

```cpp
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TrajectoryState:            # one scalar axis
    T position{}, velocity{}, acceleration{}, jerk{}   # jerk = 0 where a profile leaves it undefined

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct MotionLimits:
    T vMax{}, aMax{}               # > 0 for every limit-driven profile
    T jMax{}                       # > 0 required only by SCurveProfile
```

This item:

```cpp
enum class Degree : uint8_t { Cubic = 3, Quintic = 5 }

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BoundaryConditions:         # per axis
    T q0, qf                       # start / end position
    T v0 = 0, vf = 0               # start / end velocity
    T a0 = 0, af = 0               # start / end acceleration (quintic only)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class PolynomialProfile:
    array<T, 6>  coeff             # a0..a5 (cubic uses a0..a3, a4 = a5 = 0)
    T            duration          # tf > 0
    Degree       degree
```

## Interface

```text
PolynomialProfile(BoundaryConditions<T> bc, T tf, Degree degree)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
void Reset(BoundaryConditions<T> bc, T tf)   # keeps the degree
```

## Algorithm (pseudocode)

```text
function solveCubic(bc, tf):            # closed form, no linear solve
    a0 = bc.q0
    a1 = bc.v0
    a2 = ( 3*(bc.qf-bc.q0) - (2*bc.v0 + bc.vf)*tf) / tf^2
    a3 = (-2*(bc.qf-bc.q0) + (  bc.v0 + bc.vf)*tf) / tf^3
    a4 = a5 = 0

function solveQuintic(bc, tf):          # six matched boundary conditions
    a0 = bc.q0;  a1 = bc.v0;  a2 = bc.a0 / 2
    a3 = ( 20*Δq - (8*bc.vf + 12*bc.v0)*tf - (3*bc.a0 - bc.af)*tf^2) / (2*tf^3)
    a4 = (-30*Δq + (14*bc.vf + 16*bc.v0)*tf + (3*bc.a0 - 2*bc.af)*tf^2) / (2*tf^4)
    a5 = ( 12*Δq - ( 6*bc.vf +  6*bc.v0)*tf - (  bc.a0 - bc.af)*tf^2) / (2*tf^5)
    # Δq = qf - q0

constructor / Reset:
    degree == Degree::Cubic ? solveCubic(bc, tf) : solveQuintic(bc, tf)

function Sample(t):                     # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, duration)
    pos  = Horner(coeff, t)             # a0 + a1 t + ... + a5 t^5
    vel  = Horner(derivative(coeff), t)
    acc  = Horner(secondDerivative(coeff), t)
    jerk = Horner(thirdDerivative(coeff), t)   # 6 a3 + 24 a4 t + 60 a5 t^2
    return { pos, vel, acc, jerk }
```

## Complexity & memory

- Coefficient solve: `O(1)` once at construction (closed-form expressions).
- `Sample`: `O(degree)` — four Horner evaluations, ≤ 14 multiply-adds.
- Memory: `O(1)` — six coefficients, a duration and the degree; all stack-resident, no heap.

## Numerical / embedded notes

- Solve coefficients **once** at construction; `Sample` is then pure Horner — deterministic cycles.
- Multi-axis moves: hold one instance per axis; for many via points use `math::CubicSpline` (#61).
- Guard `tf > 0`; the `1/tf^k` terms blow up for a zero-duration segment.
- Quintic gives continuous acceleration (zero jerk endpoints); prefer it when actuators are jerk-sensitive.
- A 0 → 1 instance (e.g. `v0 = vf = 1`, `tf = 1` cubic ⇒ `s = t`) is a valid path-parameter time law
  (robotics-toolbox-cpp's `CartesianSlerpInterpolation` uses it that way).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/motion_profiles/PolynomialProfile.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class PolynomialProfile<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Uses `numerical/motion_profiles/MotionProfileTypes.hpp` (create it from the block above if this is the first
  `motion_profiles` item; header-only aggregates, no coverage `.cpp`).
- Coverage: `numerical/motion_profiles/PolynomialProfile.cpp` → `template class PolynomialProfile<float>;`
- Test: `numerical/motion_profiles/test/TestPolynomialProfile.cpp`
- Doc: `doc/motion_profiles/PolynomialProfile.md` (per `doc/TEMPLATE.md`) + row in `doc/motion_profiles/README.md`.
- CMake: `.hpp` → `target_sources(numerical.motion_profiles …)`;
  `numerical_add_coverage_sources(numerical.motion_profiles PolynomialProfile.cpp)`;
  `TestPolynomialProfile.cpp` → `numerical.motion_profiles_test`.
- New module (first `motion_profiles` item only): `numerical/motion_profiles/CMakeLists.txt` with
  `numerical_add_header_library(numerical.motion_profiles)` (links `numerical.math`), `MotionProfileTypes.hpp` in
  `target_sources`, a `test/` subdir, `add_subdirectory(motion_profiles)` in `numerical/CMakeLists.txt`,
  and a `doc/motion_profiles/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md` (step 4, new module).
