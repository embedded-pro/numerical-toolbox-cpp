# Matrix Exponential — Implementation Pseudocode

> Roadmap ref: #29 (Tier 4) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>            # static_assert(std::is_floating_point_v<T>); instantiated for float
class MatrixExponential:
    static constexpr int PadeOrder = 6         # diagonal (6,6) Padé
    static constexpr array<T, ...> padeCoeffs  # constexpr numerator/denominator coefficients
```

## Interface

```
MatrixExponential()
SquareMatrix<T, N> Compute(SquareMatrix<T, N> A)          # expm(A); hot path
SquareMatrix<T, N> Compute(SquareMatrix<T, N> A, T dt)    # expm(A*dt) convenience
```

## Algorithm (pseudocode)

```
function Compute(A):                            # OPTIMIZE_FOR_SPEED — scaling & squaring
    # 1. Scale so ‖A / 2^s‖ is small enough for Padé to be accurate
    s  = max(0, ceil(log2(InfinityNorm(A))))
    As = A * (1 / 2^s)

    # 2. Diagonal (q,q) Padé approximant of exp(As):  expm ≈ Dden⁻¹ · Nnum
    (Nnum, Dden) = PadeNumeratorDenominator(As)
    R = Solve(Dden, Nnum)                        # reuse solvers::GaussianElimination / LU

    # 3. Square s times to undo the scaling:  expm(A) = R^(2^s)
    for k in 1 .. s:
        R = R * R
    return R

function PadeNumeratorDenominator(As):
    # even/odd split halves the matrix multiplies
    A2 = As*As;  A4 = A2*A2;  A6 = A4*A2
    U  = As * (c1*I + c3*A2 + c5*A4 + ...)       # odd terms
    V  =        c0*I + c2*A2 + c4*A4 + ...        # even terms
    return (V + U, V - U)                         # (numerator, denominator)
```

## Complexity & memory

- Time: `O(N³)` per matrix multiply / solve, times `(PadeOrder + s)` products — a handful of
  `N×N` mat-muls plus one linear solve.
- Memory: a fixed set of `N×N` scratch matrices on the stack (`A2, A4, A6, U, V, R`); no heap.

## Numerical / embedded notes

- Scaling-and-squaring keeps the Padé argument small where the approximant is most accurate; the
  "hump" phenomenon (large intermediate norms) is why a naive Taylor series fails.
- Reuse `GaussianElimination`/`LU` for the `Dden⁻¹Nnum` solve — never form an explicit inverse.
- Choosing `s` from the infinity-norm is cheap and robust; over-scaling is harmless (extra squarings).
- Exploit the even/odd polynomial split to halve the number of matrix multiplies.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/MatrixExponential.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class MatrixExponential<float, 2>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/MatrixExponential.cpp` →
  `template class MatrixExponential<float, 2>;` and `<float, 3>` (the tested sizes).
- Test: `numerical/math/test/TestMatrixExponential.cpp`
- Doc: `doc/math/MatrixExponential.md` (new folder; math currently has no doc pages)
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestMatrixExponential.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
