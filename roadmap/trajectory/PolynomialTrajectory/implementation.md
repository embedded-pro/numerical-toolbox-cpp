# Polynomial Point-to-Point Trajectory — Implementation Pseudocode

> Roadmap ref: #M2 (Tier 1) · Target: `numerical/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BoundaryConditions:         # per joint
    T q0, qf                       # start / end position
    T v0 = 0, vf = 0               # start / end velocity
    T a0 = 0, af = 0               # start / end acceleration (quintic only)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TrajectoryState:
    T position, velocity, acceleration

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class PolynomialTrajectory:
    array<T, 6>  coeff             # a0..a5 (cubic uses a0..a3)
    T            duration          # tf > 0
    uint8_t      degree            # 3 = cubic, 5 = quintic
```

## Interface

```
PolynomialTrajectory(BoundaryConditions<T> bc, T tf, Degree degree)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
void Reset(BoundaryConditions<T> bc, T tf)
```

## Algorithm (pseudocode)

```
function solveCubic(bc, tf):            # closed form, no linear solve
    a0 = bc.q0
    a1 = bc.v0
    a2 = ( 3*(bc.qf-bc.q0) - (2*bc.v0 + bc.vf)*tf) / tf^2
    a3 = (-2*(bc.qf-bc.q0) + (  bc.v0 + bc.vf)*tf) / tf^3

function solveQuintic(bc, tf):          # six matched boundary conditions
    a0 = bc.q0;  a1 = bc.v0;  a2 = bc.a0 / 2
    a3 = ( 20*Δq - (8*bc.vf + 12*bc.v0)*tf - (3*bc.a0 - bc.af)*tf^2) / (2*tf^3)
    a4 = (-30*Δq + (14*bc.vf + 16*bc.v0)*tf + (3*bc.a0 - 2*bc.af)*tf^2) / (2*tf^4)
    a5 = ( 12*Δq - ( 6*bc.vf +  6*bc.v0)*tf - (  bc.a0 - bc.af)*tf^2) / (2*tf^5)
    # Δq = qf - q0

function Sample(t):                     # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, duration)
    pos = Horner(coeff, t)              # a0 + a1 t + ... + a5 t^5
    vel = Horner(derivative(coeff), t)
    acc = Horner(secondDerivative(coeff), t)
    return { pos, vel, acc }
```

## Complexity & memory

- Coefficient solve: `O(1)` once at construction (closed-form expressions).
- `Sample`: `O(degree)` — three Horner evaluations, ≤ 15 multiply-adds.
- Memory: `O(1)` — six coefficients plus a duration; all stack-resident, no heap.

## Numerical / embedded notes

- Solve coefficients **once** at construction; `Sample` is then pure Horner — deterministic cycles.
- Multi-joint moves: hold one instance per joint, or template the state on `math::Vector<T, N>`.
- Guard `tf > 0`; the `1/tf^k` terms blow up for a zero-duration segment.
- Quintic gives continuous acceleration (zero jerk endpoints); prefer it when actuators are jerk-sensitive.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/trajectory/PolynomialTrajectory.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class PolynomialTrajectory<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/trajectory/PolynomialTrajectory.cpp` →
  `template class PolynomialTrajectory<float>;`
- Test: `numerical/trajectory/test/TestPolynomialTrajectory.cpp`
- Doc: `doc/trajectory/PolynomialTrajectory.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestPolynomialTrajectory.cpp` → the `_test` target.
- New module: create `numerical/trajectory/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/CMakeLists.txt`, and add a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
