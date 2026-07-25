# Discrete Wavelet Transform (Haar / Daubechies) — Implementation Pseudocode

> Roadmap ref: #38 (Tier 4) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

A two-channel (quadrature-mirror) analysis/synthesis filter bank applied recursively per level.

```
template<typename T, std::size_t Taps>    # static_assert(std::is_floating_point_v<T>); instantiated for float
class WaveletFilters:                     # Haar (2), db2 (4), db4 (8)…
    array<T, Taps> lowAnalysis            # h  — scaling / low-pass
    array<T, Taps> highAnalysis           # g  — wavelet / high-pass, QMF of h
    array<T, Taps> lowSynthesis           # reconstruction low-pass
    array<T, Taps> highSynthesis          # reconstruction high-pass

template<typename T, std::size_t N, std::size_t Levels>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class DiscreteWaveletTransform:
    WaveletFilters<T, Taps> filters
    BoundedVector<T> coeffs               # WithMaxSize<N>: all detail bands + final approx
```

## Interface

```
DiscreteWaveletTransform(WaveletFilters<T, Taps> filters)
void   Forward(BoundedVector<T>& x, BoundedVector<T>& coeffs)   # multilevel decomposition
void   Inverse(BoundedVector<T>& coeffs, BoundedVector<T>& x)   # perfect reconstruction
size_t LevelOffset(size_t level) const                         # index of a level's cD block
```

## Algorithm (pseudocode)

```
function Forward(x, coeffs):               # OPTIMIZE_FOR_SPEED
    signal = x
    for level in 1 .. Levels:
        (cA, cD) = AnalysisStep(signal)    # filter + downsample by 2
        storeDetail(coeffs, level, cD)
        signal = cA                        # recurse on the approximation only
    storeApprox(coeffs, signal)            # final coarse approximation

function AnalysisStep(s):                   # one QMF stage, length L -> L/2 each
    for i in 0 .. L/2-1:
        cA[i] = Σ_k lowAnalysis[k]  · s[(2i + k) mod L]   # convolve + decimate
        cD[i] = Σ_k highAnalysis[k] · s[(2i + k) mod L]
    return (cA, cD)

function Inverse(coeffs, x):
    signal = finalApprox(coeffs)
    for level in Levels .. 1:              # reverse order
        cD = loadDetail(coeffs, level)
        signal = SynthesisStep(signal, cD) # upsample by 2 + filter + add
    x = signal

function SynthesisStep(cA, cD):
    return convolve(upsample2(cA), lowSynthesis) + convolve(upsample2(cD), highSynthesis)
```

## Complexity & memory

- Time: `O(N)` total for all levels (each level halves the length: `N + N/2 + N/4 + … ≤ 2N`).
- Memory: one length-`N` coefficient buffer holds every detail band plus the final approximation.
- `N` should be divisible by `2^Levels` for clean critical sampling.

## Numerical / embedded notes

- Coefficients are `constexpr` tables (Haar = `[1, 1]/√2`; Daubechies from standard scaling-coefficient
  tables). Store them once; QMF gives `g[k] = (−1)^k · h[Taps−1−k]`.
- **Boundary handling** (periodic vs symmetric extension) must match between analysis and synthesis,
  or perfect reconstruction is lost — use modulo (periodic) indexing consistently.
- Orthogonal wavelets give perfect reconstruction: `Inverse(Forward(x)) ≈ x` to rounding.
- Downsampling shrinks each successive stage, so the first level dominates the `OPTIMIZE_FOR_SPEED` cost.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/DiscreteWaveletTransform.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Forward`, and
  `extern template class DiscreteWaveletTransform<float, 16, 3>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/DiscreteWaveletTransform.cpp` →
  `template class WaveletFilters<float, 2>; template class DiscreteWaveletTransform<float, 16, 3>;`
- Test: `numerical/analysis/test/TestDiscreteWaveletTransform.cpp`
- Doc: `doc/analysis/DiscreteWaveletTransform.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestDiscreteWaveletTransform.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
