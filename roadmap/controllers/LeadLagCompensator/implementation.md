# Lead-Lag Compensator — Implementation Pseudocode

> Roadmap ref: #17 (Tier 3) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
struct LeadLagParameters:
    T gain          # K
    T zero          # z  (rad/s)
    T pole          # p  (rad/s)
    T sampleTime    # Ts (s)

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class LeadLagCompensator:
    T b0, b1        # numerator coefficients (discrete)
    T a1            # denominator coefficient (a0 normalized to 1)
    T prevInput     # x[n-1]
    T prevOutput    # y[n-1]
```

## Interface

```
LeadLagCompensator(LeadLagParameters<T> p)     # designs coefficients via Tustin
T    Compute(T input)                          # hot path
void Reset(T value = 0)
```

## Algorithm (pseudocode)

```
constructor(K, z, p, Ts):
    # Tustin (bilinear) substitution s <- (2/Ts)·(1 - z^-1)/(1 + z^-1)
    c   = 2 / Ts
    n0  = K * (c + z);   n1 = K * (z - c)      # numerator in z
    d0  = c + p;         d1 = p - c            # denominator in z
    b0  = n0 / d0;  b1 = n1 / d0;  a1 = d1 / d0   # normalize by d0

function Compute(x):                            # OPTIMIZE_FOR_SPEED
    # Direct Form I of a first-order section:
    # y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1]
    y = b0*x + b1*prevInput - a1*prevOutput
    prevInput  = x
    prevOutput = y
    return y
```

## Complexity & memory

- Time: `O(1)` per sample — three multiply-adds and two state updates.
- Memory: `O(1)` — three coefficients and two state words.

## Numerical / embedded notes

- **Lead** (`z < p`) adds phase to raise phase margin/bandwidth; **lag** (`z > p`) adds
  low-frequency gain to cut steady-state error. Same structure, different `z`/`p` ordering.
- Design (Tustin) happens once in the constructor in `float`; only the recurrence runs hot.
- Direct Form I keeps the single pole/zero numerically well-behaved; for cascaded compensators
  reuse the biquad second-order section instead of chaining first-order blocks.
- Pre-warp the corner frequencies before Tustin if exact `z`/`p` placement matters near Nyquist.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/LeadLagCompensator.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class LeadLagCompensator<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/LeadLagCompensator.cpp` →
  `template class LeadLagCompensator<float>;`
- Test: `numerical/controllers/implementations/test/TestLeadLagCompensator.cpp`
- Doc: `doc/controllers/LeadLagCompensator.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestLeadLagCompensator.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
