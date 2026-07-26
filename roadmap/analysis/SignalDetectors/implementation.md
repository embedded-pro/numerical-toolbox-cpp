# Signal Detectors — Implementation Pseudocode

> Roadmap ref: #5 (Tier 1) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Three small, independent, composable detectors sharing a scalar-state style.

```
template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class PeakHold:
    T     peak          # running max magnitude
    T     decay         # per-sample leak toward 0 in (0, 1]

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class ZeroCrossingCounter:
    T        previous   # last sample, for sign compare
    uint32_t count      # crossings since Reset
    T        hysteresis # dead-band to reject noise flips

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class RmsEnvelope:
    T  alpha            # one-pole smoothing factor on x^2
    T  meanSquare       # state = E[x^2]
```

## Interface

```
# PeakHold
PeakHold(T decay = 1)
T    Update(T x)          # hot path, returns held peak
void Reset()

# ZeroCrossingCounter
ZeroCrossingCounter(T hysteresis = 0)
bool     Update(T x)      # hot path, true when a crossing occurs
uint32_t Count() const
void     Reset()

# RmsEnvelope
RmsEnvelope(T alpha)
T    Update(T x)          # hot path, returns sqrt(meanSquare)
void Reset(T value = 0)
```

## Algorithm (pseudocode)

```
function PeakHold.Update(x):            # OPTIMIZE_FOR_SPEED
    m = abs(x)
    peak = max(m, peak * decay)         # leak first, then admit new peak
    return peak

function ZeroCrossingCounter.Update(x): # OPTIMIZE_FOR_SPEED
    crossed = (sign(x) != sign(previous)) and (abs(x) > hysteresis)
    if crossed: count += 1
    previous = x
    return crossed

function RmsEnvelope.Update(x):         # OPTIMIZE_FOR_SPEED
    meanSquare += alpha * (x*x - meanSquare)   # one-pole on x^2 (reuses item 1)
    return sqrt(meanSquare)
```

## Complexity & memory

- Time: `O(1)` per sample per detector (peak: 1 mul + max; ZC: 1 compare; RMS: 1 MAC + 1 sqrt).
- Memory: `O(1)` — one to two state words each; no buffers, no window.

## Numerical / embedded notes

- `decay = 1` is a true infinite peak-hold; `decay < 1` gives an exponential release (VU-meter feel).
- Zero-crossing `sign()` must treat `0` consistently; the `hysteresis` dead-band stops a signal
  hovering near zero from inflating the count. Fundamental frequency ≈ `count / (2·window·Ts)`.
- `RmsEnvelope` feeds the item-1 one-pole with `x²` and returns the square root of the smoothed
  mean-square (reuses `ExponentialMovingAverage`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/SignalDetectors.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on each `Update`, and
  `extern template class PeakHold<float>; ZeroCrossingCounter<float>; RmsEnvelope<float>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/SignalDetectors.cpp` →
  `template class PeakHold<float>; template class ZeroCrossingCounter<float>; template class RmsEnvelope<float>;`
- Test: `numerical/analysis/test/TestSignalDetectors.cpp`
- Doc: `doc/analysis/SignalDetectors.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSignalDetectors.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
