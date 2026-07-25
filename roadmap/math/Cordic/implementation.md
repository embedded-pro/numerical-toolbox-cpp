# CORDIC — Implementation Pseudocode

> Roadmap ref: #23 (Tier 3) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Iterations = 16>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class Cordic:
    static constexpr array<T, Iterations> atanTable   # atan(2^-i), constexpr
    static constexpr T K = 0.6072529350...            # rotation-mode gain 1/∏√(1+2^-2i)
    struct SinCos { T sin; T cos; }
```

No per-call heap: the angle table is a `constexpr std::array` living in flash/ROM.

## Interface

```
Cordic()
SinCos       SineCosine(T angleRadians)      # rotation mode; hot path
T            Arctangent2(T y, T x)           # vectoring mode -> atan2
T            Magnitude(T y, T x)             # vectoring mode -> hypot
Vector<T, 2> Rotate(Vector<T, 2> v, T angle) # rotate a 2-vector
```

## Algorithm (pseudocode)

```
function SineCosine(angle):                  # OPTIMIZE_FOR_SPEED — rotation mode
    fold angle into [-π/2, π/2] (remember the quadrant sign)
    (x, y, z) = (K, 0, angle)                # prescale x by the gain K once
    for i in 0 .. Iterations-1:
        d = (z >= 0) ? +1 : -1               # steer z -> 0
        (x, y) = (x - d*(y >> i), y + d*(x >> i))   # shift-add, no multiply
        z = z - d * atanTable[i]
    return { sin: y, cos: x } with quadrant sign applied

function Arctangent2(y, x):                   # vectoring mode
    resolve the quadrant from the signs of x, y
    z = 0
    for i in 0 .. Iterations-1:
        d = (y >= 0) ? -1 : +1               # steer y -> 0
        (x, y) = (x - d*(y >> i), y + d*(x >> i))
        z = z - d * atanTable[i]
    return z (+ quadrant offset)

function Magnitude(y, x):
    run the vectoring iterations; return K * final_x   # |(x,y)| after gain compensation
```

## Complexity & memory

- `O(Iterations)` per call — only shifts and adds, **no multiplier**; a fixed, data-independent
  cycle count (ideal for deterministic real-time paths).
- Memory: `O(Iterations)` `constexpr` table shared by all instances; `O(1)` working state.

## Numerical / embedded notes

- Accuracy gains ≈ one correct bit per iteration; `Iterations = 16` ≈ 16-bit precision.
- Convergence domain is `|angle| ≤ ~1.7433` (Σ atan 2^-i); fold inputs into range first and apply
  the quadrant sign afterwards.
- Prescale by the constant gain `K ≈ 0.6073` once (rotation mode) rather than per iteration.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/Cordic.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `SineCosine`, and
  `extern template class Cordic<float, 16>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/Cordic.cpp` → `template class Cordic<float, 16>;`
  (add `Cordic<float, 8>` for the iteration-scaling test).
- Test: `numerical/math/test/TestCordic.cpp`
- Doc: `doc/math/Cordic.md` (new folder; math currently has no doc pages)
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestCordic.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
