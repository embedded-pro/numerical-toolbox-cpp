# Convolution & Correlation — Implementation Pseudocode

> Roadmap ref: #11 (Tier 2) · Target: `numerical/analysis` · Namespace `analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

Free functions over bounded vectors; no persistent state. Outputs are written into a caller-owned
`infra::BoundedVector` sized at compile time.

```
template<typename T, std::size_t N>  # static_assert(std::is_floating_point_v<T>); instantiated for float
using Vector = infra::BoundedVector<T>::WithMaxSize<N>

# Output sizes (compile-time):
#   linear conv/corr of (M, K)  -> M + K - 1
#   circular conv of length N   -> N
```

## Interface

```
# Every function: template<typename T, std::size_t ...>  # instantiated for float

# Convolution
void LinearConvolution(Vector<T,M> x, Vector<T,K> h, Vector<T,M+K-1>& y)     # |y| = |x|+|h|-1
void CircularConvolution(Vector<T,N> x, Vector<T,N> h, Vector<T,N>& y)       # equal length N

# Correlation
void CrossCorrelation(Vector<T,M> x, Vector<T,K> y, Vector<T,M+K-1>& r)      # lag-indexed
void AutoCorrelation(Vector<T,M> x, Vector<T,2*M-1>& r)                      # r = CrossCorrelation(x, x)
size_t ArgMaxLag(Vector<T,N> r)                                             # delay / lag estimate

# Optional fast path (power-of-two)
void FastConvolution(Vector<T,M> x, Vector<T,K> h, Vector<T,L>& y, Fft<T>& fft)
```

## Algorithm (pseudocode)

```
function LinearConvolution(x, h, y):            # OPTIMIZE_FOR_SPEED
    for n in 0 .. |x|+|h|-2:
        acc = 0
        for k in max(0, n-|h|+1) .. min(n, |x|-1):
            acc += x[k] * h[n-k]                 # MAC in wide accumulator
        y[n] = acc

function CircularConvolution(x, h, y):          # length N, indices mod N
    for n in 0 .. N-1:
        acc = 0
        for k in 0 .. N-1:
            acc += x[k] * h[(n-k) mod N]
        y[n] = acc

function CrossCorrelation(x, y, r):
    # r[l] = sum_n x[n]·y[n-l]  == convolution with a time-reversed y
    LinearConvolution(x, reverse(y), r)

function FastConvolution(x, h, y, fft):
    L = next_pow2(|x| + |h| - 1)
    X = fft.Forward(zeroPad(x, L));  H = fft.Forward(zeroPad(h, L))
    Y = X .* H                                  # pointwise complex multiply
    y = fft.Inverse(Y)[0 .. |x|+|h|-2]
```

## Complexity & memory

- Direct linear/circular: `O(M·K)` time, `O(1)` extra memory (writes into caller buffer).
- FFT fast convolution: `O(L log L)` — worth it when both operands are long; needs three length-`L`
  complex scratch buffers (compile-time sized).
- Correlation is convolution with one operand reversed — identical cost.

## Numerical / embedded notes

- Optionally divide auto-correlation by `r[0]` to get a normalized correlation in `[-1, 1]`.
- Circular convolution equals linear convolution only when both signals are zero-padded to
  `|x|+|h|-1`; document this to avoid time-aliasing surprises.
- `ArgMaxLag` of the cross-correlation gives an integer sample delay — the basis of TDOA/lag estimation.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/analysis/ConvolutionCorrelation.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `LinearConvolution`/`CircularConvolution`,
  and `extern template` declarations of the `float` instantiations under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/analysis/ConvolutionCorrelation.cpp` → explicit `float` function instantiations,
  e.g. `template void LinearConvolution<float, M, K>(...);` for each free function.
- Test: `numerical/analysis/test/TestConvolutionCorrelation.cpp`
- Doc: `doc/analysis/ConvolutionCorrelation.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestConvolutionCorrelation.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
