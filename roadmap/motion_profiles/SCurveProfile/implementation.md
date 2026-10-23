# S-Curve (Jerk-Limited) Profile — Implementation Pseudocode

> Roadmap ref: #60 (Tier 2) · Target: `numerical/motion_profiles` · Namespace `motion_profiles` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

`TrajectoryState<T>` and `MotionLimits<T>` come from `numerical/motion_profiles/MotionProfileTypes.hpp`
(canonical definition: PolynomialProfile spec → Data structures); not redefined here. This profile
requires `vMax`, `aMax`, `jMax > 0`.

```cpp
template<typename T>               # static_assert(std::is_floating_point_v<T>); instantiated for float
class SCurveProfile:
    T q0, direction, jMax
    T Tj, Ta, Tv                   # jerk sub-phase, whole accel phase, cruise
    array<T, 7> segT               # duration of each of the 7 phases
    array<T, 8> segStart           # start time of each phase (+ tf)
    array<T, 8> segPos, segVel, segAcc   # state at each phase boundary
    T tf
    bool reachesMaxAccel, reachesMaxVel
```

## Interface

```text
SCurveProfile(T q0, T qf, MotionLimits<T> limits)
TrajectoryState<T> Sample(T t)      # hot path
T    Duration()
bool ReachesMaxAccel() / ReachesMaxVel()
```

## Algorithm (pseudocode)

Rest-to-rest closed form (Biagiotti & Melchiorri §3.4, `v0 = v1 = 0`), `h = |qf − q0|`:

```cpp
# Seven phases: [+j][a=const][-j][v=const][-j][a=const][+j]
function plan(q0, qf, lim):
    h = |qf - q0|;  direction = sign(qf - q0);  jMax = lim.jMax
    if h == 0: all durations 0; return
    # Step 1 — assume vMax is reached
    if lim.vMax * lim.jMax >= lim.aMax^2:        # aMax reached on the way to vMax
        Tj = lim.aMax / lim.jMax
        Ta = Tj + lim.vMax / lim.aMax
    else:                                        # triangular accel, peak jMax·Tj < aMax
        Tj = sqrt(lim.vMax / lim.jMax)
        Ta = 2*Tj
    Tv = h / lim.vMax - Ta
    if Tv < 0:
        # Step 2 — vMax not reached (Tv = 0), assume aMax still reached
        Tv = 0
        Tj = lim.aMax / lim.jMax
        Δ  = lim.aMax^4 / lim.jMax^2 + 4*lim.aMax*h
        Ta = (lim.aMax^2 / lim.jMax + sqrt(Δ)) / (2*lim.aMax)
        if Ta < 2*Tj:
            # Step 3 — neither aMax nor vMax reached
            Tj = cbrt(h / (2*lim.jMax))
            Ta = 2*Tj
    aLim = jMax*Tj;  vLim = aLim*(Ta - Tj)       # peak |acc|, peak |vel|
    reachesMaxAccel = (Ta > 2*Tj);  reachesMaxVel = (Tv > 0)
    segT = [Tj, Ta-2*Tj, Tj, Tv, Tj, Ta-2*Tj, Tj]            # tf = 2·Ta + Tv
    jerkOfPhase = [+jMax, 0, -jMax, 0, -jMax, 0, +jMax]
    segPos[0] = segVel[0] = segAcc[0] = 0
    for i in 0..6:                               # exact constant-jerk integration
        τ = segT[i];  j = jerkOfPhase[i]
        segPos[i+1] = segPos[i] + segVel[i]*τ + segAcc[i]*τ^2/2 + j*τ^3/6
        segVel[i+1] = segVel[i] + segAcc[i]*τ + j*τ^2/2
        segAcc[i+1] = segAcc[i] + j*τ
        segStart[i+1] = segStart[i] + τ

function Sample(t):                              # OPTIMIZE_FOR_SPEED
    t = clamp(t, 0, tf)
    i = last phase with segStart[i] ≤ t (i ≤ 6);  tau = t - segStart[i]
    j = jerkOfPhase[i]
    acc = segAcc[i] + j*tau
    vel = segVel[i] + segAcc[i]*tau + 0.5*j*tau^2
    s   = segPos[i] + segVel[i]*tau + 0.5*segAcc[i]*tau^2 + (1/6)*j*tau^3
    return { q0 + direction*s, direction*vel, direction*acc, direction*j }
```

Branch consistency (checked numerically on 20 000 random `h, vMax, aMax, jMax`): every branch ends at `h`
with zero velocity and acceleration, `|v| ≤ vMax`, `|a| ≤ aMax`, `|j| = jMax`; no phase duration is
negative (`Ta − 2Tj = vMax/aMax − aMax/jMax ≥ 0` in step 1a; the step-2 test guards it otherwise).

## Complexity & memory

- Planning: `O(1)` — at most three closed-form candidates (one `sqrt`, one `cbrt`), no iteration.
- `Sample`: `O(1)` — locate one of seven phases, evaluate a cubic-in-time (Horner).
- Memory: `O(1)` — four 8-entry tables plus seven durations; stack only, no heap.

## Numerical / embedded notes

- Precompute the seven durations and boundary states **once**; each tick is one cubic evaluation.
- Short moves collapse segments to zero length (`Tv = 0`, `Ta = 2·Tj`) — never a negative duration.
- Bounded jerk means acceleration is **continuous** ⇒ far less vibration than a trapezoid.
- Profile is symmetric about its midpoint for rest-to-rest moves (`Td = Ta`).
- **Synchronization:** multi-axis moves finish together by uniform time scaling of every non-slowest
  axis by `λ = T_sync / T_axis ≥ 1`: velocity, acceleration and jerk become `v/λ, a/λ², j/λ³`, sampled at
  `t/λ`. Replanning with limits `(vMax/λ, aMax/λ², jMax/λ³)` gives exactly `Duration() = λ·T_axis` in every
  branch (branch tests and the step formulas are invariant under this scaling) — planning stays `O(1)`.
- **Non-rest boundary velocities** (`v0, v1 ≠ 0`, B&M §3.4 general case) need the iterative `aMax`
  reduction (`aMax ← γ·aMax`, `0 < γ < 1`) plus a feasibility pre-check; not covered — future work.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/motion_profiles/SCurveProfile.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class SCurveProfile<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Uses `numerical/motion_profiles/MotionProfileTypes.hpp` (create it if this is the first `motion_profiles` item).
- Coverage: `numerical/motion_profiles/SCurveProfile.cpp` → `template class SCurveProfile<float>;`
- Test: `numerical/motion_profiles/test/TestSCurveProfile.cpp`
- Doc: `doc/motion_profiles/SCurveProfile.md` (per `doc/TEMPLATE.md`) + row in `doc/motion_profiles/README.md`.
- CMake: `.hpp` → `target_sources(numerical.motion_profiles …)`;
  `numerical_add_coverage_sources(numerical.motion_profiles SCurveProfile.cpp)`;
  `TestSCurveProfile.cpp` → `numerical.motion_profiles_test`.
- New module (first `motion_profiles` item only): `numerical/motion_profiles/CMakeLists.txt` with
  `numerical_add_header_library(numerical.motion_profiles)`, `MotionProfileTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(motion_profiles)` in `numerical/CMakeLists.txt`, and a `doc/motion_profiles/` folder.
- Depends on: #56 (`TrapezoidalProfile`, shares the synchronization scheme).
- Generic pattern: see `roadmap/DEPLOYMENT.md` (step 4, new module).
