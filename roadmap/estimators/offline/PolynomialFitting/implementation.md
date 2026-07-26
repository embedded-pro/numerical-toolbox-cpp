# Polynomial Least-Squares Fitting — Implementation Pseudocode

> Roadmap ref: #12 (Tier 2) · Target: `numerical/estimators/offline` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Samples, std::size_t Degree>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class PolynomialFitting:                                # degree d ⇒ d+1 coefficients
    Matrix<T, Degree+1, 1> coefficients                 # [c0, c1, ..., cd]
    static_assert Samples >= Degree + 1                 # enough points to fit
```

## Interface

```
PolynomialFitting()
void Fit(Matrix<T, Samples, 1> x, Matrix<T, Samples, 1> y)   # hot path
T    Predict(T x)                                            # Horner evaluation
const Matrix<T, Degree+1, 1>& Coefficients()
```

## Algorithm (pseudocode)

```
function Fit(x, y):                          # OPTIMIZE_FOR_SPEED
    # Build Vandermonde design matrix V (Samples x Degree+1)
    for i in 0 .. Samples-1:
        V[i,0] = 1
        for j in 1 .. Degree:
            V[i,j] = V[i,j-1] * x[i]         # incremental power, no pow()
    # Normal equations:  (Vᵀ V) c = Vᵀ y
    A = Vᵀ * V                               # (Degree+1) x (Degree+1), symmetric PD
    b = Vᵀ * y
    coefficients = solvers::SolveSystem(A, b)          # Gaussian elimination
    # (or)  L = CholeskyDecomposition(A);  solve L Lᵀ c = b

function Predict(x):                          # Horner
    acc = coefficients[Degree]
    for j in Degree-1 .. 0:
        acc = acc * x + coefficients[j]
    return acc
```

## Complexity & memory

- Fit: `O(Samples·Degree²)` to form `VᵀV`, `O(Degree³)` to solve — both bounded at compile time.
- Predict: `O(Degree)` via Horner (one MAC per term).
- Memory: `O(Degree²)` for the normal-equation matrix on the stack; no heap.

## Numerical / embedded notes

- `VᵀV` is symmetric positive-definite ⇒ `CholeskyDecomposition` is ~2× cheaper than Gaussian
  elimination and preserves symmetry; fall back to pivoted elimination if conditioning is poor.
- The Vandermonde system is **ill-conditioned** for high `Degree`; keep `Degree ≤ 4–5` and
  **center/scale** the abscissa (`x ← (x − x̄)/σ`) before fitting to tame the condition number.
- Build powers incrementally (`V[i,j] = V[i,j-1]·x[i]`) to avoid `pow()` and reduce round-off.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/estimators/offline/PolynomialFitting.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Fit`, and
  `extern template class PolynomialFitting<float, 8, 2>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/offline/PolynomialFitting.cpp` → `template class PolynomialFitting<float, 8, 2>;`
- Test: `numerical/estimators/offline/test/TestPolynomialFitting.cpp`
- Doc: `doc/estimators/PolynomialFitting.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestPolynomialFitting.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
