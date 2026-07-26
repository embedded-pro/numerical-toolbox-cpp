# Hilbert Transform / Analytic Signal — Implementation Pseudocode

> Roadmap ref: #37 (Tier 4) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Two interchangeable back-ends behind one interface (OCP): an FFT method (block, exact) and an FIR
method (streaming, approximate).

```
template<typename T, std::size_t N>      # static_assert(std::is_floating_point_v<T>); instantiated for float
class AnalyticSignalFft:
    FastFourierTransform<T>& fft
    BoundedVector<Complex<T>> spectrum   # WithMaxSize<N>
    BoundedVector<Complex<T>> analytic   # WithMaxSize<N>

template<typename T, std::size_t Taps>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class HilbertFir:                        # Type III/IV antisymmetric FIR
    array<T, Taps>          coeff        # ≈ 2/(π k) for odd k, 0 for even k (windowed)
    RecursiveBuffer<T, Taps> delay       # linear-phase delay line
```

## Interface

```
# FFT (block) analytic signal
BoundedVector<Complex<T>>& Analytic(BoundedVector<T>& x)   # returns x + j·H{x}

# FIR (streaming) Hilbert
Complex<T> Filter(T x)                    # hot path: real in, analytic out (group-delayed)

# Feature extraction (either back-end)
T InstantaneousAmplitude(Complex<T> a)    # |a| (envelope)
T InstantaneousPhase(Complex<T> a)        # atan2(Im, Re)
T InstantaneousFrequency(T phaseNow, T phasePrev, T Ts)   # unwrapped dφ/dt
```

## Algorithm (pseudocode)

```
function Analytic(x):                      # OPTIMIZE_FOR_SPEED — Marple FFT method
    X = fft.Forward(x)                     # length N
    # Build one-sided spectrum: keep DC & Nyquist, double positives, zero negatives
    H[0]   = X[0]
    H[N/2] = X[N/2]                        # N even
    for k in 1     .. N/2-1: H[k] = 2·X[k]
    for k in N/2+1 .. N-1:   H[k] = 0
    analytic = fft.Inverse(H)              # real = x, imag = Hilbert{x}
    return analytic

function Filter(x):                        # FIR streaming approximation
    delay.Push(x)
    imag = dot(coeff, delay)               # 90° phase shift via antisymmetric taps
    real = delay[centerTap]                # matched group-delay copy of the input
    return Complex(real, imag)

function InstantaneousFrequency(phaseNow, phasePrev, Ts):
    dphi = wrapToPi(phaseNow - phasePrev)
    return dphi / (2π·Ts)
```

## Complexity & memory

- FFT method: `O(N log N)` per block, exact; two length-`N` complex buffers.
- FIR method: `O(Taps)` per sample, `O(Taps)` state; constant latency `(Taps−1)/2` samples.
- Feature extraction is `O(1)` per sample (`atan2`, magnitude, one difference).

## Numerical / embedded notes

- The FFT method needs the block windowed (or periodic) to limit edge artefacts; the FIR method
  streams continuously but only approximates the ideal `−90°` shift across its passband.
- Instantaneous **phase must be unwrapped** before differencing, else frequency spikes by `2π`.
- FIR Hilbert taps are antisymmetric with every even tap zero — exploit that to halve the MACs.
- Reuses `FastFourierTransform` / `RealFft` for the block back-end.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/HilbertTransform.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Analytic`/`Filter`, and
  `extern template class AnalyticSignalFft<float, 64>; HilbertFir<float, 31>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/HilbertTransform.cpp` →
  `template class AnalyticSignalFft<float, 64>; template class HilbertFir<float, 31>;`
- Test: `numerical/analysis/test/TestHilbertTransform.cpp`
- Doc: `doc/analysis/HilbertTransform.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestHilbertTransform.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
