# Trapezoidal (LSPB) Velocity Profile — Implementation Pseudocode

> Roadmap ref: #M3 (Tier 1) · Target: `numerical/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct MotionLimits:
    T vMax           # peak velocity  (> 0)
    T aMax           # peak accel     (> 0)

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TrajectoryState:
    T position, velocity, acceleration

template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class TrapezoidalProfile:
    T q0, direction                # start, sign(qf - q0)
    T vPeak, aMax                  # cruise velocity actually reached
    T tAccel, tCruise, tf          # phase boundaries (blend, flat, total)
```

## Interface

```
TrapezoidalProfile(T q0, T qf, MotionLimits<T> limits)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
bool IsTriangular()                 # true when vMax is never reached
```

## Algorithm (pseudocode)

```
function plan(q0, qf, limits):
    d   = |qf - q0|;  direction = sign(qf - q0)
    dBlend = limits.vMax^2 / limits.aMax        # distance used by accel + decel
    if d >= dBlend:                             # trapezoid: cruise phase exists
        vPeak   = limits.vMax
        tAccel  = vPeak / limits.aMax
        tCruise = (d - dBlend) / vPeak
    else:                                       # triangle: peak below vMax
        vPeak   = sqrt(d * limits.aMax)
        tAccel  = vPeak / limits.aMax
        tCruise = 0
    tf = 2*tAccel + tCruise

function Sample(t):                             # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, tf)
    if t < tAccel:                              # parabolic ramp-up
        acc = aMax;  vel = aMax*t;        s = 0.5*aMax*t^2
    else if t < tAccel + tCruise:              # linear cruise
        acc = 0;     vel = vPeak;         s = vPeak*(t - 0.5*tAccel)
    else:                                       # parabolic ramp-down
        td  = tf - t
        acc = -aMax; vel = aMax*td;       s = d - 0.5*aMax*td^2
    return { q0 + direction*s, direction*vel, direction*acc }
```

## Complexity & memory

- Planning: `O(1)` — one `sqrt` and a branch to pick trapezoid vs triangle.
- `Sample`: `O(1)` — one phase branch, a couple of multiply-adds.
- Memory: `O(1)` — six scalars; entirely stack-resident, no heap.

## Numerical / embedded notes

- Precompute the three phase boundaries **once**; `Sample` then reduces to one branch per tick.
- Velocity is continuous but acceleration is **discontinuous** at blend joins (bounded jerk spikes) —
  upgrade to `SCurveProfile` when that excites structural modes.
- Guard degenerate inputs: `d == 0` ⇒ zero-length profile; `aMax == 0` or `vMax == 0` ⇒ reject.
- Symmetric ramps mean the decel phase mirrors accel — reuse the same `aMax`, flip the sign.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/trajectory/TrapezoidalProfile.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class TrapezoidalProfile<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/trajectory/TrapezoidalProfile.cpp` →
  `template class TrapezoidalProfile<float>;`
- Test: `numerical/trajectory/test/TestTrapezoidalProfile.cpp`
- Doc: `doc/trajectory/TrapezoidalProfile.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestTrapezoidalProfile.cpp` → the `_test` target.
- New module: create `numerical/trajectory/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/CMakeLists.txt`, and add a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
