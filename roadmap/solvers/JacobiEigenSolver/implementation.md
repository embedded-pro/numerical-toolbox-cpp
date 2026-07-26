# Symmetric Eigenvalue Solver (Cyclic Jacobi) — Implementation Pseudocode

> Roadmap ref: #42 (Tier 5) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>            # static_assert(std::is_floating_point_v<T>); instantiated for float
class JacobiEigenSolver:
    Vector<T, N>        eigenvalues
    Matrix<T, N, N>     eigenvectors             # columns
    std::size_t         maxSweeps
    T                   tolerance                # off-diagonal threshold
    bool                converged
```

## Interface

```
JacobiEigenSolver(std::size_t maxSweeps = 30, T tolerance = T(1e-9))
bool Decompose(const Matrix<T,N,N>& symmetricA)     # hot path
const Vector<T,N>&   Eigenvalues() const
const Matrix<T,N,N>& Eigenvectors() const
```

## Algorithm (pseudocode)

```
function Decompose(A):                          # OPTIMIZE_FOR_SPEED  (cyclic Jacobi)
    assert IsSymmetric(A)
    D = A;  V = Identity
    for sweep in 1 .. maxSweeps:
        if OffDiagonalNorm(D) < tolerance: converged = true; break
        for p in 0 .. N-2:                      # cyclic order over all pairs
            for q in p+1 .. N-1:
                if |D[p,q]| ≈ 0: continue
                theta  = (D[q,q] − D[p,p]) / (2·D[p,q])      # angle that zeros D[p,q]
                t      = sign(theta) / (|theta| + sqrt(theta² + 1))
                c      = 1 / sqrt(t² + 1);  s = t·c
                D = Jᵀ(p,q,c,s) · D · J(p,q,c,s)             # updates two rows & two cols
                V = V · J(p,q,c,s)                           # accumulate eigenvectors
    eigenvalues  = diag(D)
    eigenvectors = V
    SortDescending(eigenvalues, eigenvectors)
    return converged
```

## Complexity & memory

- `O(N³)` per sweep; convergence is quadratic, typically `6–10` sweeps ⇒ `O(N³)` in practice.
- Each rotation updates only two rows/columns — `O(N)` work per off-diagonal pair.
- Memory: `O(N²)` for `D` and `V` — static, no heap.

## Numerical / embedded notes

- **Symmetric input only** — assert symmetry; the algorithm guarantees **real** eigenvalues and an
  **orthonormal** eigenvector set (`V` stays orthogonal because every `J` is a rotation).
- **Cyclic** sweeps visit pairs in fixed order, avoiding the `O(N²)` largest-off-diagonal search of
  classical Jacobi while keeping the same accuracy — a good fit for small on-device matrices.
- Highly accurate even for clustered eigenvalues; preferred over QR-iteration for the small symmetric
  matrices typical on-device (covariance, inertia tensors, Gramians).
- Reused by `SingularValueDecomposition` (the `AᵀA` route) and the symmetric `LyapunovSylvester` path.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/JacobiEigenSolver.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Decompose`, and
  `extern template class JacobiEigenSolver<float, 3>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/JacobiEigenSolver.cpp` →
  `template class JacobiEigenSolver<float, 3>;` and `<float, 2>` (the tested sizes).
- Test: `numerical/solvers/test/TestJacobiEigenSolver.cpp`
- Doc: `doc/solvers/JacobiEigenSolver.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestJacobiEigenSolver.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
