# Lyapunov / Sylvester Equation Solvers — Implementation Pseudocode

> Roadmap ref: #31 (Tier 4) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N, std::size_t M>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SylvesterSolver:                          # solves A X + X B = C
    # continuous Lyapunov:  B = Aᵀ,  C = −Q,  M = N
    # discrete   Lyapunov:  A X Aᵀ − X = −Q
    Matrix<T, N, M> X                            # solution
    bool solvable
```

## Interface

```
SylvesterSolver()
bool SolveSylvester(A[N×N], B[M×M], C[N×M])                # hot path
bool SolveContinuousLyapunov(A[N×N], Q[N×N])   # A X + X Aᵀ = −Q
bool SolveDiscreteLyapunov(A[N×N], Q[N×N])     # A X Aᵀ − X = −Q
const Matrix<T,N,M>& Solution() const
```

## Algorithm (pseudocode)

```
function SolveSylvester(A, B, C):               # OPTIMIZE_FOR_SPEED  (Bartels-Stewart)
    # 1. reduce to (quasi-)triangular form via real Schur / eigen-decomposition
    (Ua, Ra) = Schur(A)                         # reuse QrDecomposition sweeps
    (Ub, Rb) = Schur(B)
    C̃ = Uaᵀ · C · Ub                            # transform the right-hand side
    # 2. back-substitute column by column (columns decouple once triangular)
    for j in 0 .. M-1:
        rhs      = C̃[:,j] − Σ_{k<j} Rb[k,j]·X̃[:,k]
        X̃[:,j]   = GaussianElimination(Ra + Rb[j,j]·I, rhs)
    # 3. back-transform
    X = Ua · X̃ · Ubᵀ
    return solvability from the column solves

function SolveContinuousLyapunov(A, Q):         # symmetric fast path
    (U, lambda) = JacobiEigenSolver(A)          # A symmetric ⇒ real eigenpairs
    Q̃ = Uᵀ · (−Q) · U
    X̃[i,j] = Q̃[i,j] / (lambda[i] + lambda[j])   # divide by eigenvalue sums
    return X = U · X̃ · Uᵀ
```

## Complexity & memory

- `O(N³ + M³)` for the Schur/eigen reductions plus `O(N²M)` back-substitution.
- Memory: `O(N² + M²)` for the transforms and the solution — static, no heap.

## Numerical / embedded notes

- **Solvability:** Sylvester is uniquely solvable iff `A` and `−B` share no eigenvalue; continuous
  Lyapunov needs `λ_i + λ_j ≠ 0`; discrete needs `λ_i·λ_j ≠ 1`. Flag the degenerate case.
- Reuse `JacobiEigenSolver` for the symmetric Lyapunov fast path and `GaussianElimination` for the
  per-column solves; a full general Schur reuses `QrDecomposition` sweeps.
- A **stable** `A` yields a symmetric positive-definite Lyapunov solution `X` — the certificate that
  proves stability and forms the controllability/observability **Gramians** (item 26).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/solvers/LyapunovSylvester.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on the solvers, and
  `extern template class SylvesterSolver<float, 2, 2>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/LyapunovSylvester.cpp` →
  `template class SylvesterSolver<float, 2, 2>;` and `<float, 3, 3>` (the tested sizes).
- Test: `numerical/solvers/test/TestLyapunovSylvester.cpp`
- Doc: `doc/solvers/LyapunovSylvester.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestLyapunovSylvester.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
