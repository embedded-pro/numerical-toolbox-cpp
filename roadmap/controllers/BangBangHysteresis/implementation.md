# Bang-Bang / Hysteresis (Relay) Controller — Implementation Pseudocode

> Roadmap ref: #4 (Tier 1) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
enum RelayState: Low, High

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class BangBangHysteresis:
    T          lowThreshold      # switch-off point
    T          highThreshold     # switch-on point, high > low
    T          outputLow         # value emitted in Low state
    T          outputHigh        # value emitted in High state
    RelayState state = Low
```

## Interface

```
BangBangHysteresis(T lowThreshold, T highThreshold,
                   T outputLow, T outputHigh)
T          Update(T measurement)        # hot path
void       Reset(RelayState initial = Low)
RelayState State() const
```

## Algorithm (pseudocode)

```
function Update(x):                      # OPTIMIZE_FOR_SPEED
    # Schmitt trigger: transitions only at the band edges
    if state == Low and x >= highThreshold:
        state = High
    else if state == High and x <= lowThreshold:
        state = Low
    return (state == High) ? outputHigh : outputLow
```

## Complexity & memory

- Time: `O(1)` per sample — at most two comparisons and a select.
- Memory: `O(1)` — four thresholds/levels plus a one-bit state.

## Numerical / embedded notes

- The dead-band `[lowThreshold, highThreshold]` sets the switching hysteresis; a wider band
  trades tracking accuracy for fewer switching events (relay chatter).
- Signal path is comparison-only (no arithmetic), so the relay introduces no numerical error.
- Switching frequency is bounded by the band width and the signal slope; size the band from the
  actuator's minimum on/off time to protect relays and power stages.
- `really_assert(highThreshold > lowThreshold)` in the constructor to reject an inverted band
  that would latch the output.
- Deterministic branchless variant: precompute `outputHigh−outputLow` and blend by the state bit
  to avoid a data-dependent branch in tight loops.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/BangBangHysteresis.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Update`, and
  `extern template class BangBangHysteresis<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/BangBangHysteresis.cpp` →
  `template class BangBangHysteresis<float>;`
- Test: `numerical/controllers/implementations/test/TestBangBangHysteresis.cpp`
- Doc: `doc/controllers/BangBangHysteresis.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestBangBangHysteresis.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
