# CIC (Cascaded Integrator-Comb) Filter — Implementation Pseudocode

> Roadmap ref: #14 (Tier 2) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Stages, std::size_t R, std::size_t M>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class CicDecimator:                        # R = decimation, M = differential delay
    array<T, Stages>           integrator  # integrator running sums
    array<Comb<M>, Stages>     comb         # each holds an M-sample delay line
    counter                    phase = 0    # 0..R-1 decimation phase
```

## Interface

```
CicDecimator()
optional<T> Filter(T input)              # returns a sample once every R inputs
void Reset()
static constexpr T Gain()               # (R*M)^Stages, for output scaling
```

## Algorithm (pseudocode)

```
function Filter(x):                        # OPTIMIZE_FOR_SPEED
    # --- integrator section, runs at the INPUT rate ---
    acc = x
    for i in 0..Stages-1:
        integrator[i] += acc               # y = y + acc  (pure accumulation)
        acc = integrator[i]

    phase = phase + 1
    if phase < R: return none              # decimate: emit 1 of every R inputs

    phase = 0
    # --- comb section, runs at the OUTPUT (decimated) rate ---
    for i in 0..Stages-1:
        delayed = comb[i].PushPop(acc)     # value M output-samples ago
        acc     = acc - delayed            # differencing
    return acc / Gain()                    # normalize DC gain toward 1
```

## Complexity & memory

- Time: `O(Stages)` adds at input rate + `O(Stages)` subs per `R` inputs — **no multipliers**.
- Memory: `O(Stages · M)` accumulator/delay words.

## Deployment

- Header: `numerical/filters/passive/CicFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class CicDecimator<float, Stages, R, M>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/CicFilter.cpp` →
  `template class CicDecimator<float, Stages, R, M>;`
- Test: `numerical/filters/passive/test/TestCicFilter.cpp`
- Doc: `doc/filters/passive/CicFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestCicFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

## Numerical / embedded notes

- DC gain is `(R·M)^Stages`; divide the comb output by this factor (`Gain()`) to normalize toward
  unity.
- Passband droop (`sinc^Stages`) usually needs a small FIR compensator downstream.
- The interpolator is the dual: comb → zero-stuff (×R) → integrator, in that order.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.
