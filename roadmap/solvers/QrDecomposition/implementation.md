# QR Decomposition (Householder / Givens) — Implementation Pseudocode

> Roadmap ref: #27 (Tier 4) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Rows, std::size_t Cols>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class QrDecomposition:
    Matrix<T, Rows, Cols> qr        # reflectors below diagonal, R on/above diagonal
    Vector<T, Cols>       betas     # Householder scaling per column
    static_assert Rows >= Cols
```

## Interface

```
QrDecomposition()
bool Decompose(const Matrix<T,Rows,Cols>& A)      # hot path; false if rank-deficient
Matrix<T,Rows,Cols> Q() const                     # thin Q (accumulated on request)
Matrix<T,Cols,Cols> R() const                     # upper triangular
Vector<T,Cols> SolveLeastSquares(const Vector<T,Rows>& b) const
void ApplyQtranspose(Vector<T,Rows>& v) const     # v ← Qᵀ v  (no explicit Q)
```

## Algorithm (pseudocode)

```
function Decompose(A):                                 # OPTIMIZE_FOR_SPEED
    qr = A
    for k in 0 .. Cols-1:
        x       = qr[k.., k]                           # sub-column below diagonal
        v, beta = Householder(x)                       # reflector zeroing x below x[0]
        for j in k .. Cols-1:                          # apply H = I − beta·v·vᵀ to trailing block
            qr[k.., j] -= beta · v · (vᵀ · qr[k.., j])
        store v (below diag), betas[k]
        if |qr[k,k]| < eps: return false               # rank-deficient
    return true

function SolveLeastSquares(b):                          # OPTIMIZE_FOR_SPEED
    c = ApplyQtranspose(b)                              # c = Qᵀ b
    return BackSubstitute(R, c[0..Cols-1])             # R x = c₁  (upper-triangular solve)

# Givens option (streaming): rotate one new row into R with Cols plane rotations,
# zeroing each sub-diagonal entry — an O(Cols²) rank-1 update, no full re-factor.
```

## Complexity & memory

- Householder factorization: `O(Rows·Cols²)` (≈ `2·Rows·Cols² − (2/3)Cols³`).
- Accumulating `Q` is optional — `ApplyQtranspose` avoids forming it for a solve.
- Memory: `O(Rows·Cols)` for the packed factors — static, no heap.

## Numerical / embedded notes

- **Householder is backward stable** and preferred for dense `A`; **Givens** wins for sparse or
  *streaming* updates (adding one row/column) because each rotation touches only two rows.
- Least squares via QR (`R x = Qᵀb`) is far better conditioned than the normal equations
  `AᵀA x = Aᵀb` (whose condition number is squared).
- Foundational: reused by `SingularValueDecomposition`, the symmetric-eigen building blocks, and the
  square-root Kalman filter; reuse the triangular back-substitution shared with `GaussianElimination`.
- Fix the sign of each Householder pivot (`−sign(x₀)·‖x‖`) to avoid cancellation.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/QrDecomposition.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Decompose`/`SolveLeastSquares`, and
  `extern template class QrDecomposition<float, 4, 3>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/QrDecomposition.cpp` →
  `template class QrDecomposition<float, 4, 3>;` and `<float, 3, 3>` (the tested shapes).
- Test: `numerical/solvers/test/TestQrDecomposition.cpp`
- Doc: `doc/solvers/QrDecomposition.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestQrDecomposition.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
