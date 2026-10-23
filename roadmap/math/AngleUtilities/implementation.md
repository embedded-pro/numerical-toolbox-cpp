# Angle Utilities (wrap, shortest difference, unwrap) — Implementation Pseudocode

> Roadmap ref: #54 (Tier 1) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

Every consumer that integrates or compares angles needs the same four operations: wrapping into a
canonical interval, the shortest signed difference between two angles, and unwrapping a wrapped
measurement back into a continuous signal. Today `ComplementaryFilter` carries a private wrap and
robotics-toolbox-cpp's analytical IK, mobile-base odometry and heading controllers would each write
their own. One header, one convention.

## Data structures

None — stateless free function templates.

## Interface

```cpp
template<typename T> T WrapToPi(T angle)                      # (−π, π]; hot path
template<typename T> T WrapToTwoPi(T angle)                   # [0, 2π)
template<typename T> T AngleDifference(T target, T current)   # WrapToPi(target − current), short way
template<typename T> T UnwrapAngle(T previousUnwrapped, T wrapped)   # continuous continuation
# each: static_assert(std::is_floating_point_v<T>, "...")
```

## Algorithm (pseudocode)

```text
twoPi = 2·std::numbers::pi_v<T>

function WrapToPi(a):                          # OPTIMIZE_FOR_SPEED
    r = math::Fmod(a + π, twoPi)               # C fmod keeps the sign of the dividend: r ∈ (−2π, 2π)
    if r ≤ 0: r = r + twoPi                    # r ∈ (0, 2π]
    return r − π                               # (−π, π]; a = ±π ⇒ +π

function WrapToTwoPi(a):
    r = math::Fmod(a, twoPi)
    if r < 0: r = r + twoPi
    return r ≥ twoPi ? 0 : r                   # rounding of r + 2π can land exactly on 2π

function AngleDifference(target, current):
    return WrapToPi(target − current)

function UnwrapAngle(previousUnwrapped, wrapped):
    return previousUnwrapped + AngleDifference(wrapped, previousUnwrapped)
```

## Complexity & memory

- `O(1)` each: one `fmod`, one compare, one add. No state, no heap.

## Numerical / embedded notes

- **Convention (normative):** `WrapToPi` returns `(−π, π]` — `−π` maps to `+π` — so a joint at exactly
  half a turn has one representation. `ComplementaryFilter`'s private wrap returns `[−π, π)`; switching it
  to `WrapToPi` changes the result only at exactly `±π`.
- Precision degrades with `|angle|`: `fmod` is exact, but `2π` is not representable, so the error grows
  like `|angle|·ε`. Wrap at the source (each integration step) rather than after long accumulation.
- `UnwrapAngle` assumes consecutive samples differ by less than `π`; sample faster than half a turn per
  period.
- Use `math::Fmod` (platform-dispatched) rather than `std::fmod`, so the ARM implementation is picked up.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/math/AngleUtilities.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `WrapToPi`, and
  `extern template float WrapToPi<float>(float);` (and the other three) under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/AngleUtilities.cpp` → `template float WrapToPi<float>(float);` and the others.
- Test: `numerical/math/test/TestAngleUtilities.cpp`
- Doc: `doc/math/AngleUtilities.md` (per `doc/TEMPLATE.md`) + row in `doc/math/README.md`
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestAngleUtilities.cpp` → `numerical.math_test`.
- Follow-up: replace `ComplementaryFilter`'s private wrap with `WrapToPi`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
