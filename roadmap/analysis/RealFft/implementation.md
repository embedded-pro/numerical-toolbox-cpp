# Real-Input FFT (RFFT) — Implementation Pseudocode

> Roadmap ref: #25 (Tier 3) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Wraps the existing complex `FastFourierTransform`. A length-`N` real transform is done with one
length-`N/2` complex FFT plus a split (untangle) pass.

```
template<typename T, std::size_t N>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class RealFft:                             # N even, power of two
    FastFourierTransform<T>& fft           # injected complex engine (operates at size N/2)
    BoundedVector<Complex<T>> packed        # WithMaxSize<N/2>, the half-size buffer
    BoundedVector<Complex<T>> twiddles      # WithMaxSize<N/2>, W_N^k precomputed
    BoundedVector<Complex<T>> spectrum      # WithMaxSize<N/2 + 1>, output bins 0..N/2
```

## Interface

```
RealFft(FastFourierTransform<T>& engine)                          # DIP: inject the complex FFT
BoundedVector<Complex<T>>& Forward(BoundedVector<T>& realInput)   # returns N/2+1 unique bins
BoundedVector<T>&          Inverse(BoundedVector<Complex<T>>& halfSpectrum)  # back to N reals
```

## Algorithm (pseudocode)

```
function Forward(x):                        # OPTIMIZE_FOR_SPEED, |x| = N
    # 1. Pack even/odd real samples into a half-length complex sequence
    for k in 0 .. N/2-1:
        packed[k] = Complex(x[2k], x[2k+1])

    # 2. One half-size complex FFT
    Z = fft.Forward(packed)                 # length N/2

    # 3. Split/untangle Z into the true spectrum X[0..N/2]
    spectrum[0]   = Complex(Z[0].Re + Z[0].Im, 0)   # DC (real)
    spectrum[N/2] = Complex(Z[0].Re - Z[0].Im, 0)   # Nyquist (real)
    for k in 1 .. N/2-1:
        Ze = 0.5·(Z[k] + conj(Z[N/2-k]))    # even part
        Zo = 0.5·(Z[k] - conj(Z[N/2-k]))    # odd part
        spectrum[k] = Ze + (-j)·twiddles[k]·Zo      # recombine with W_N^k
    return spectrum

function Inverse(halfSpectrum):
    # mirror the split using Hermitian symmetry, one N/2 inverse complex FFT, then unpack
    ...symmetric reverse of Forward...
```

## Complexity & memory

- Time: `O(N log N)` but ≈ **half** the butterflies and twiddles of a complex FFT on the same `N`.
- Memory: a half-size complex buffer `N/2` plus an `N/2+1` output — ~2× smaller than a complex FFT.
- Only bins `0 … N/2` are stored; the rest follow from Hermitian symmetry `X[N−k] = conj(X[k])`.

## Numerical / embedded notes

- Reuses the existing `FastFourierTransform` by dependency injection — no duplicate butterfly code (DRY).
- `N` must be even and a power of two so the underlying complex FFT stays radix-2.
- Twiddles `W_N^k = e^{−j2πk/N}` are `constexpr`/precomputed; store only `N/2` of them.
- Store DC and Nyquist as real-only — their imaginary parts are exactly zero.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/RealFft.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Forward`/`Inverse`, and
  `extern template class RealFft<float, 16>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/RealFft.cpp` → `template class RealFft<float, 16>;`
- Test: `numerical/analysis/test/TestRealFft.cpp`
- Doc: `doc/analysis/RealFft.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestRealFft.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
