# Trapezoidal (LSPB) Velocity Profile — Implementation Pseudocode

> Roadmap ref: #56 (Tier 1) · Target: `numerical/motion_profiles` · Namespace `motion_profiles` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

`TrajectoryState<T>` and `MotionLimits<T>` come from `numerical/motion_profiles/MotionProfileTypes.hpp`
(canonical definition: PolynomialProfile spec → Data structures); not redefined here. This profile
reads `vMax`, `aMax` and ignores `jMax`.

```cpp
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class TrapezoidalProfile:
    T q0, direction, distance      # start, sign(qf - q0), |qf - q0|
    T vPeak, aMax                  # cruise velocity actually reached, ramp acceleration
    T tAccel, tCruise, tf          # phase boundaries (blend, flat, total)
```

## Interface

```cpp
TrapezoidalProfile(T q0, T qf, MotionLimits<T> limits)                  # minimum-time plan
static std::optional<TrapezoidalProfile> PlanWithDuration(T q0, T qf, T aMax, T duration)
                                                                        # fixed-time plan (synchronization)
TrajectoryState<T> Sample(T t)      # hot path; jerk = 0
T    Duration()
bool IsTriangular()                 # true when no cruise phase exists
```

## Algorithm (pseudocode)

```cpp
function plan(q0, qf, limits):
    distance = |qf - q0|;  direction = sign(qf - q0);  aMax = limits.aMax
    dBlend = limits.vMax^2 / limits.aMax        # distance used by accel + decel
    if distance >= dBlend:                      # trapezoid: cruise phase exists
        vPeak   = limits.vMax
        tAccel  = vPeak / aMax
        tCruise = (distance - dBlend) / vPeak
    else:                                       # triangle: peak below vMax
        vPeak   = sqrt(distance * aMax)
        tAccel  = vPeak / aMax
        tCruise = 0
    tf = 2*tAccel + tCruise

function PlanWithDuration(q0, qf, aMax, duration):
    # d = v·(duration - v/aMax)  ⇒  v² - aMax·duration·v + aMax·d = 0, take the smaller root
    d    = |qf - q0|
    disc = aMax^2 * duration^2 - 4*aMax*d
    if aMax <= 0 or duration <= 0 or disc < 0:  # feasible iff aMax·duration² ≥ 4d
        return nullopt
    vPeak   = 2*aMax*d / (aMax*duration + sqrt(disc))   # = (aMax·duration − sqrt(disc))/2, cancellation-free
    tAccel  = vPeak / aMax
    tCruise = duration - 2*tAccel
    tf      = duration
    return profile{ q0, sign(qf - q0), d, vPeak, aMax, tAccel, tCruise, tf }

function Sample(t):                             # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, tf)
    if t < tAccel:                              # parabolic ramp-up
        acc = aMax;  vel = aMax*t;        s = 0.5*aMax*t^2
    else if t < tAccel + tCruise:              # linear cruise
        acc = 0;     vel = vPeak;         s = vPeak*(t - 0.5*tAccel)
    else:                                       # parabolic ramp-down
        td  = tf - t
        acc = -aMax; vel = aMax*td;       s = distance - 0.5*aMax*td^2
    return { q0 + direction*s, direction*vel, direction*acc, 0 }
```

## Complexity & memory

- Planning (both variants): `O(1)` — one `sqrt` and a branch.
- `Sample`: `O(1)` — one phase branch, a couple of multiply-adds.
- Memory: `O(1)` — eight scalars; entirely stack-resident, no heap.

## Numerical / embedded notes

- Precompute the three phase boundaries **once**; `Sample` then reduces to one branch per tick.
- Velocity is continuous but acceleration is **discontinuous** at blend joins (bounded jerk spikes) —
  upgrade to `SCurveProfile` when that excites structural modes.
- Guard degenerate inputs: `distance == 0` ⇒ zero-length profile; `aMax <= 0` or `vMax <= 0` ⇒ reject.
- **Synchronization:** multi-axis moves finish together by uniform time scaling of every non-slowest
  axis by `λ = T_sync / T_axis ≥ 1` (`T_sync` = largest `Duration()`): its velocity, acceleration and
  jerk become `v/λ, a/λ², j/λ³` and it is sampled at `t/λ`. Replanning that axis with limits
  `(vMax/λ, aMax/λ²)` yields exactly `Duration() = λ·T_axis` (same branch, same shape).
  `PlanWithDuration(q0, qf, aMax, T_sync)` is the alternative that keeps `aMax` and only lowers the cruise
  speed; for `T_sync ≥` the minimum-time duration its `vPeak ≤ vMax` (the root decreases with duration).
- `PlanWithDuration` uses the conjugate form of the smaller root — the textbook
  `(aMax·T − sqrt(disc))/2` cancels catastrophically in `float` when `4·aMax·d ≪ aMax²·T²`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/motion_profiles/TrapezoidalProfile.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class TrapezoidalProfile<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Uses `numerical/motion_profiles/MotionProfileTypes.hpp` (create it if this is the first `motion_profiles` item).
- Coverage: `numerical/motion_profiles/TrapezoidalProfile.cpp` → `template class TrapezoidalProfile<float>;`
- Test: `numerical/motion_profiles/test/TestTrapezoidalProfile.cpp`
- Doc: `doc/motion_profiles/TrapezoidalProfile.md` (per `doc/TEMPLATE.md`) + row in `doc/motion_profiles/README.md`.
- CMake: `.hpp` → `target_sources(numerical.motion_profiles …)`;
  `numerical_add_coverage_sources(numerical.motion_profiles TrapezoidalProfile.cpp)`;
  `TestTrapezoidalProfile.cpp` → `numerical.motion_profiles_test`.
- New module (first `motion_profiles` item only): `numerical/motion_profiles/CMakeLists.txt` with
  `numerical_add_header_library(numerical.motion_profiles)`, `MotionProfileTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(motion_profiles)` in `numerical/CMakeLists.txt`, and a `doc/motion_profiles/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md` (step 4, new module).
