# Continuous-to-Discrete (c2d) — Implementation Pseudocode

> Roadmap ref: #30 (Tier 4) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
enum class DiscretizationMethod { ZeroOrderHold, Tustin, ForwardEuler, BackwardEuler }

template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
class ContinuousToDiscrete:      # static_assert(std::is_floating_point_v<T>); instantiated for float
    MatrixExponential<T, StateSize + InputSize> expm               # for the ZOH block
```

Both input and output reuse `LinearTimeInvariant<T, StateSize, InputSize, OutputSize>`
(the `A`, `B`, `C`, `D` matrices).

## Interface

```
ContinuousToDiscrete()
LinearTimeInvariant<...> Convert(LinearTimeInvariant<...> sys, T Ts,
                                 DiscretizationMethod method)      # hot path
```

## Algorithm (pseudocode)

```
function Convert(sys, Ts, method):              # OPTIMIZE_FOR_SPEED
    switch method:
      ZeroOrderHold: return Zoh(sys, Ts)
      Tustin:        return Bilinear(sys, Ts)
      ForwardEuler:  return { A: I + A*Ts, B: B*Ts, C, D }
      BackwardEuler: return Backward(sys, Ts)

function Zoh(sys, Ts):
    # Van Loan augmented-matrix trick:
    #   M = [[A, B],[0, 0]] * Ts ;  expm(M) = [[Ad, Bd],[0, I]]
    M  = block([[A, B],[0, 0]]) * Ts
    E  = expm.Compute(M)                         # reuse MatrixExponential (item 29)
    Ad = E.block(0, 0, StateSize, StateSize)
    Bd = E.block(0, StateSize, StateSize, InputSize)
    return { Ad, Bd, C, D }                       # C, D unchanged under ZOH

function Bilinear(sys, Ts):                       # Tustin:  s <- (2/Ts)(z-1)/(z+1)
    a  = 2 / Ts
    P  = Solve(a*I - A, I)                         # reuse GaussianElimination (no explicit inverse)
    Ad = P * (a*I + A)
    Bd = P * B * 2
    Cd = C * P * a
    Dd = D + C * P * B
    return { Ad, Bd, Cd, Dd }

function Backward(sys, Ts):
    P = Solve(I - A*Ts, I)
    return { Ad: P, Bd: P*B*Ts, Cd: C*P, Dd: D + C*P*B*Ts }
```

## Complexity & memory

- ZOH: one `(n+m)×(n+m)` matrix exponential — `O((n+m)³)`.
- Tustin / Euler: one `n×n` solve plus a few mat-muls — `O(n³)`.
- Memory: the augmented matrix plus a few `n×n` scratch blocks on the stack; no heap.

## Numerical / embedded notes

- ZOH is **exact** for piecewise-constant inputs (real DAC/PWM behaviour) — the default choice.
- Tustin (bilinear) maps the stable left-half-plane into the unit disk, preserving stability, and is
  the right choice for controller/filter discretization; add frequency **prewarping** when a specific
  cutoff must be matched.
- Forward Euler is cheapest but can push poles outside the unit circle for large `Ts` (conditionally
  stable); backward Euler is unconditionally stable but adds phase lag.
- Never invert explicitly — factor once with `GaussianElimination`/`LU` and reuse the factorization.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/ContinuousToDiscrete.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Convert`, and
  `extern template class ContinuousToDiscrete<float, 2, 1, 1>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/ContinuousToDiscrete.cpp` →
  `template class ContinuousToDiscrete<float, 2, 1, 1>;`
- Test: `numerical/math/test/TestContinuousToDiscrete.cpp`
- Doc: `doc/math/ContinuousToDiscrete.md` (new folder; math currently has no doc pages)
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestContinuousToDiscrete.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
