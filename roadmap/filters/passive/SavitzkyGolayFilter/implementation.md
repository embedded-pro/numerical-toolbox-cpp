# Savitzky-Golay Filter — Implementation Pseudocode

> Roadmap ref: #22 (Tier 3) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv = 0>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SavitzkyGolay:                                # Window odd, Order < Window
    static constexpr array<T, Window> coeffs = SgKernel<Window, Order, Deriv>()
    RecursiveBuffer<T, Window> line                 # last Window samples
```

## Interface

```
SavitzkyGolay(T initial = 0)
T    Filter(T input)          # smoothed centre value (or derivative estimate)
void Reset(T value = 0)
static constexpr array<T, Window> Coefficients()
```

## Algorithm (pseudocode)

```
# Compile-time kernel from a local least-squares polynomial fit:
#   A      = Vandermonde of window offsets [-h..+h], h = (Window-1)/2
#   C      = (Aᵀ A)⁻¹ Aᵀ                       # projection matrix
#   coeffs = row `Deriv` of C  × (Deriv! / Ts^Deriv)   # smoothing or derivative kernel
function SgKernel<Window, Order, Deriv>() -> array<T, Window>:   # constexpr

function Filter(x):                        # OPTIMIZE_FOR_SPEED
    line.Push(x)
    acc = 0
    for k in 0..Window-1:
        acc += coeffs[k] * line[k]         # symmetric FIR convolution
    return acc                             # centre estimate at lag (Window-1)/2
```

## Complexity & memory

- Time: `O(Window)` multiply-accumulates per sample.
- Memory: `O(Window)` delay line + a `constexpr` coefficient table in ROM/flash.

## Numerical / embedded notes

- Coefficients come from a *fixed* least-squares fit, so precompute them as `constexpr` (the classic
  tables are integer numerators over a common divisor).
- Symmetric kernel ⇒ linear phase, group delay `(Window−1)/2`.
- Preserves peak height, width, and higher moments far better than a moving average.
- Derivative kernels fold the `1/Ts^Deriv` factor into the (offline) coefficient scale.
- Choose `Order < Window`: higher order tracks curvature (less peak distortion) but smooths noise
  less.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/SavitzkyGolayFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class SavitzkyGolay<float, Window, Order, Deriv>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/SavitzkyGolayFilter.cpp` →
  `template class SavitzkyGolay<float, Window, Order, Deriv>;`
- Test: `numerical/filters/passive/test/TestSavitzkyGolayFilter.cpp`
- Doc: `doc/filters/passive/SavitzkyGolayFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSavitzkyGolayFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
