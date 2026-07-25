# S-Curve (Jerk-Limited) Profile — Implementation Pseudocode

> Roadmap ref: #M9 (Tier 2) · Target: `numerical/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct MotionLimits:
    T vMax, aMax, jMax             # velocity, acceleration, jerk ceilings (> 0)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TrajectoryState:
    T position, velocity, acceleration, jerk

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class SCurveProfile:
    T q0, direction
    array<T, 7> segT               # duration of each of the 7 phases
    array<T, 8> segPos, segVel, segAcc   # state at each phase boundary
    T tf
```

## Interface

```
SCurveProfile(T q0, T qf, MotionLimits<T> limits)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
bool ReachesMaxAccel() / ReachesMaxVel()
```

## Algorithm (pseudocode)

```
# Seven phases: [+j][a=const][-j][v=const][-j][a=const][+j]
function plan(q0, qf, lim):
    d = |qf - q0|;  direction = sign(qf - q0)
    # --- acceleration sub-phase durations (rest-to-rest) ---
    if (lim.vMax * lim.jMax) >= lim.aMax^2:      # aMax is reached
        Tj = lim.aMax / lim.jMax
        Ta = Tj + lim.vMax / lim.aMax
    else:                                        # triangular accel (aMax not reached)
        Tj = sqrt(lim.vMax / lim.jMax)
        Ta = 2*Tj
    # --- constant-velocity phase ---
    Tv = d / lim.vMax - Ta                       # shrink/kill accel phase if Tv < 0
    if Tv < 0: recompute Ta, Tj for a short move (no cruise)
    segT = [Tj, Ta-2*Tj, Tj, Tv, Tj, Ta-2*Tj, Tj]
    integrate boundary states segPos/segVel/segAcc from constant-jerk kinematics

function Sample(t):                              # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, tf)
    i, tau = locateSegment(t)                    # phase index + local time
    j = jerkOfPhase(i)                           # +jMax, 0, or -jMax
    acc = segAcc[i] + j*tau
    vel = segVel[i] + segAcc[i]*tau + 0.5*j*tau^2
    s   = segPos[i] + segVel[i]*tau + 0.5*segAcc[i]*tau^2 + (1/6)*j*tau^3
    return { q0 + direction*s, direction*vel, direction*acc, direction*j }
```

## Complexity & memory

- Planning: `O(1)` — a fixed set of algebraic phase-duration formulas, no iteration.
- `Sample`: `O(1)` — locate one of seven phases, evaluate a cubic-in-time (Horner).
- Memory: `O(1)` — three 8-entry boundary tables plus seven durations; stack only, no heap.

## Numerical / embedded notes

- Precompute the seven durations and boundary states **once**; each tick is one cubic evaluation.
- Handle the degenerate cases: short moves where `aMax` and/or `vMax` are never reached collapse
  segments to zero length — never emit a negative duration.
- Bounded jerk means acceleration is **continuous** ⇒ far less vibration than a trapezoid; this is
  the reason to pay for the extra bookkeeping.
- Profile is symmetric about its midpoint for rest-to-rest moves — reuse the accel table, mirrored.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/trajectory/SCurveProfile.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class SCurveProfile<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/trajectory/SCurveProfile.cpp` →
  `template class SCurveProfile<float>;`
- Test: `numerical/trajectory/test/TestSCurveProfile.cpp`
- Doc: `doc/trajectory/SCurveProfile.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSCurveProfile.cpp` → the `_test` target.
- New module: create `numerical/trajectory/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/CMakeLists.txt`, and add a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
