# Singular Value Decomposition (Golub-Kahan) — Implementation Pseudocode

> Roadmap ref: #43 (Tier 5) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Rows, std::size_t Cols>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SingularValueDecomposition:
    Matrix<T, Rows, Cols> U          # left singular vectors (thin)
    Vector<T, Cols>       sigma      # singular values, descending
    Matrix<T, Cols, Cols> V          # right singular vectors
    static_assert Rows >= Cols
```

## Interface

```
SingularValueDecomposition()
bool Decompose(const Matrix<T,Rows,Cols>& A)         # hot path
const Vector<T,Cols>& SingularValues() const
Matrix<T,Cols,Rows> PseudoInverse(T tol) const
std::size_t Rank(T tol) const
T ConditionNumber() const
Vector<T,Cols> SolveLeastSquares(const Vector<T,Rows>& b) const
```

## Algorithm (pseudocode)

```
function Decompose(A):                          # OPTIMIZE_FOR_SPEED  (Golub-Kahan)
    # Phase 1 — bidiagonalize with alternating Householder reflectors (reuse QrDecomposition)
    (U, B, V) = HouseholderBidiagonalize(A)     # B upper-bidiagonal
    # Phase 2 — implicit-shift QR sweeps on B (Golub-Reinsch)
    repeat:
        for each split point: deflate if a superdiagonal ≈ 0
        apply Givens rotations to chase the off-diagonal down B    # updates U, V
    until B is diagonal within tolerance
    sigma = |diag(B)|;  fold signs into U
    SortDescending(sigma, U, V)
    return converged

function PseudoInverse(tol):                     # A⁺ = V Σ⁺ Uᵀ
    for i: sInv[i] = (sigma[i] > tol) ? 1/sigma[i] : 0    # threshold tiny σ
    return V · diag(sInv) · Uᵀ

function ConditionNumber():  return sigma_max / sigma_min
```

## Complexity & memory

- Bidiagonalization dominates: `O(Rows·Cols² + Cols³)`; the QR sweeps add `O(Cols²)` per iteration.
- Memory: `O(Rows·Cols)` for `U` plus `O(Cols²)` for `V` — static, no heap.

## Numerical / embedded notes

- Reuse **Householder** reflectors (item 27) for the bidiagonalization and **Givens** rotations for
  the sweeps; because `σ² = eig(AᵀA)`, results cross-check against `JacobiEigenSolver`.
- The **pseudo-inverse** thresholds tiny singular values to zero — this is what makes least squares
  robust for rank-deficient or ill-conditioned `A` (regularization by truncation).
- `ConditionNumber = σ_max/σ_min` is the definitive conditioning metric; `Rank` counts `σ > tol`.
- For wide matrices (`Rows < Cols`) decompose `Aᵀ` and swap the roles of `U` and `V`.
- Foundational: powers `TotalLeastSquares`, pseudo-inverse control allocation, and model reduction.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/SingularValueDecomposition.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Decompose`, and
  `extern template class SingularValueDecomposition<float, 4, 3>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/SingularValueDecomposition.cpp` →
  `template class SingularValueDecomposition<float, 4, 3>;` and `<float, 3, 3>` (the tested shapes).
- Test: `numerical/solvers/test/TestSingularValueDecomposition.cpp`
- Doc: `doc/solvers/SingularValueDecomposition.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSingularValueDecomposition.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
