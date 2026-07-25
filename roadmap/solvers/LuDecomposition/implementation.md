# LU Decomposition with Partial Pivoting — Implementation Pseudocode

> Roadmap ref: #28 (Tier 4) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>            # static_assert(std::is_floating_point_v<T>); instantiated for float
class LuDecomposition:
    Matrix<T, N, N>           lu                # L (unit, below diag) + U (on/above diag)
    std::array<std::size_t,N> piv               # row permutation P
    int                       pivotSign         # +1 / −1 for the determinant
    bool                      singular
```

## Interface

```
LuDecomposition()
bool  Decompose(const Matrix<T,N,N>& A)              # PA = LU; hot path; false if singular
Vector<T,N>   Solve(const Vector<T,N>& b) const      # hot path
Matrix<T,N,N> Inverse() const                        # N column solves
T Determinant() const
```

## Algorithm (pseudocode)

```
function Decompose(A):                          # OPTIMIZE_FOR_SPEED  (GEPP)
    lu = A;  piv = [0..N-1];  pivotSign = +1
    for k in 0 .. N-1:
        p = argmax_{i>=k} |lu[i,k]|             # partial pivot: largest magnitude
        if |lu[p,k]| < eps: singular = true; return false
        if p != k: swap rows k,p; swap piv[k],piv[p]; pivotSign = −pivotSign
        for i in k+1 .. N-1:
            lu[i,k] /= lu[k,k]                  # multiplier stored in L
            lu[i, k+1..] -= lu[i,k] · lu[k, k+1..]     # update trailing submatrix
    return true

function Solve(b):                              # OPTIMIZE_FOR_SPEED
    y = ForwardSubstitute(L, P·b)               # unit-lower solve
    return BackSubstitute(U, y)                 # upper solve

function Determinant():  return pivotSign · Π_k lu[k,k]
function Inverse():      solve A·X = I column by column   # avoid unless truly needed
```

## Complexity & memory

- `Decompose`: `O(N³)` (≈ `(2/3)N³`); each `Solve`: `O(N²)`; `Inverse`: `O(N³)` (N solves).
- Memory: `O(N²)` for the packed `LU` plus `O(N)` for the pivot vector — static, no heap.

## Numerical / embedded notes

- **Partial pivoting** (largest-magnitude pivot) bounds the growth factor and makes GEPP backward
  stable; never skip it for a general matrix.
- Reuse the factored form and triangular solves shared with `GaussianElimination` — factor **once**,
  then solve cheaply for many right-hand sides.
- `Determinant` from the product of `U`'s diagonal (times the pivot sign) avoids cofactor blow-up.
- Prefer `CholeskyDecomposition` when `A` is symmetric positive-definite (≈2× faster, no pivoting);
  use LU for general / non-symmetric systems.
- Only materialize `Inverse()` when the inverse itself is the product — otherwise `Solve` is cheaper
  and better conditioned.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/LuDecomposition.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Decompose`/`Solve`, and
  `extern template class LuDecomposition<float, 3>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/LuDecomposition.cpp` →
  `template class LuDecomposition<float, 3>;` and `<float, 4>` (the tested sizes).
- Test: `numerical/solvers/test/TestLuDecomposition.cpp`
- Doc: `doc/solvers/LuDecomposition.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestLuDecomposition.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
