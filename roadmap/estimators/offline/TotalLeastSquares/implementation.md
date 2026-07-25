# Total Least Squares — Implementation Pseudocode

> Roadmap ref: #44 (Tier 5) · Target: `numerical/estimators/offline` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Samples, std::size_t Features>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class TotalLeastSquares:
    Vector<T, Features> coefficients
    static_assert Samples >= Features + 1
```

## Interface

```
TotalLeastSquares()
bool Fit(Matrix<T, Samples, Features> A, Vector<T, Samples> b)   # hot path; false if degenerate
T    Predict(Vector<T, Features> x)
const Vector<T, Features>& Coefficients()
```

## Algorithm (pseudocode)

```
function Fit(A, b):                          # OPTIMIZE_FOR_SPEED
    # Stack the augmented system  M = [A | b]   (Samples x Features+1)
    M = concat_columns(A, b)
    # Thin SVD:  M = U Σ Vᵀ   (reuse solvers::SingularValueDecomposition)
    (U, sigma, V) = SVD(M)
    # Smallest singular value -> its right-singular vector v (length Features+1)
    v = V.column(Features)                   # column of smallest σ
    denom = v[Features]                       # last entry
    if |denom| < eps: return false           # b lies in span(A): no TLS solution
    for i in 0 .. Features-1:
        coefficients[i] = -v[i] / denom       # partition  x = −v₁ / v₂
    return true

function Predict(x):
    return dot(coefficients, x)
```

## Complexity & memory

- Fit: dominated by the SVD of the `Samples × (Features+1)` augmented matrix,
  `O(Samples·Features²)` (Golub-Kahan bidiagonalization + implicit-QR sweeps).
- Memory: `O(Samples·Features)` for `M` and the SVD factors on the stack; no heap.

## Numerical / embedded notes

- TLS minimizes the **orthogonal** (perpendicular) distance to the fit, unlike ordinary least
  squares which minimizes vertical residuals — the correct model when the regressors `A` are noisy.
- The solution is the right-singular vector of the **smallest** singular value; a tiny gap between
  the two smallest σ signals an ill-posed problem (near-degenerate solution).
- Column-scale `A` and `b` to comparable magnitudes first, otherwise the SVD is dominated by the
  largest column and the TLS/OLS distinction is lost.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/estimators/offline/TotalLeastSquares.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Fit`, and
  `extern template class TotalLeastSquares<float, 8, 1>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/offline/TotalLeastSquares.cpp` →
  `template class TotalLeastSquares<float, 8, 1>; template class TotalLeastSquares<float, 10, 2>;`
- Test: `numerical/estimators/offline/test/TestTotalLeastSquares.cpp`
- Doc: `doc/estimators/TotalLeastSquares.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestTotalLeastSquares.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
