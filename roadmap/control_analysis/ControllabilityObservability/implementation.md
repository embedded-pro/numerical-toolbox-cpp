# Controllability / Observability Matrices & Gramians — Implementation Pseudocode

> Roadmap ref: #26 (Tier 3) · Target: `numerical/control_analysis` · Namespace `control_analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t n, std::size_t m, std::size_t p>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ControllabilityObservability:
    # Built from a discrete plant math::LinearTimeInvariant<T, n, m, p> (A,B,C,D).
    # Controllability matrix  Wc = [B  AB  A²B ... A^{n-1}B]   -> n × (n·m)
    # Observability   matrix  Wo = [C; CA; CA²; ...; CA^{n-1}] -> (n·p) × n
    using Ctrb = math::Matrix<T, n, n*m>
    using Obsv = math::Matrix<T, n*p, n>
    using Gramian = math::SquareMatrix<T, n>      # symmetric n × n
```

## Interface

```
# Structural matrices:
static Ctrb ControllabilityMatrix(const LinearTimeInvariant& plant)
static Obsv ObservabilityMatrix (const LinearTimeInvariant& plant)

# Rank tests (numerical rank with relative tolerance):
static bool IsControllable(const LinearTimeInvariant& plant, T tol = T(1e-6))
static bool IsObservable  (const LinearTimeInvariant& plant, T tol = T(1e-6))
static size_t Rank(const MatrixLike& M, T tol)

# Energy Gramians (require A Schur-stable; solved via discrete Lyapunov, item 31):
static Gramian ControllabilityGramian(const LinearTimeInvariant& plant)
static Gramian ObservabilityGramian  (const LinearTimeInvariant& plant)
```

## Algorithm (pseudocode)

```
function ControllabilityMatrix(A, B):             # OPTIMIZE_FOR_SPEED
    block = B                                      # A⁰B
    Wc.SetBlock(block, row=0, col=0)
    for k in 1 .. n-1:
        block = A * block                          # A^k B, one matmul reuses A^{k-1}B
        Wc.SetBlock(block, row=0, col=k*m)
    return Wc

function ObservabilityMatrix(A, C):
    block = C                                      # C A⁰
    Wo.SetBlock(block, row=0, col=0)
    for k in 1 .. n-1:
        block = block * A                          # C A^k
        Wo.SetBlock(block, row=k*p, col=0)
    return Wo

function Rank(M, tol):
    # Gaussian elimination with partial pivoting; count pivots above tol·|largest pivot|
    reduce M to row-echelon; return count of non-negligible pivots

function IsControllable(plant, tol):  return Rank(ControllabilityMatrix(plant), tol) == n
function IsObservable  (plant, tol):  return Rank(ObservabilityMatrix(plant),   tol) == n

function ControllabilityGramian(plant):
    # discrete Lyapunov:  A·Wc·Aᵀ − Wc + B·Bᵀ = 0
    return SolveDiscreteLyapunov(plant.A, plant.B * Transpose(plant.B))
```

## Complexity & memory

- Matrix build: `O(n³·m)` / `O(n³·p)` — `n−1` block products, each `A·block` costs `O(n²·m)`.
- Rank test: `O(n³)` Gaussian elimination on the stacked matrix.
- Gramians: cost of the Lyapunov solver (item 31), typically `O(n³)`.
- Memory: `O(n²·max(m,p))` for the stacked matrices — all `std::array`-backed, stack/static.

## Numerical / embedded notes

- **Duality** — observability of `(A, C)` equals controllability of `(Aᵀ, Cᵀ)`; implement one Krylov
  routine and feed it the dual to avoid duplicating logic (DRY).
- Rank is tolerance-sensitive: scale `tol` by the largest pivot (or matrix norm), not an absolute value.
- Companion/Krylov stacking becomes ill-conditioned as `n` grows; fine for the small orders typical on an
  MCU, but prefer a staircase/SVD rank for large `n`.
- Gramians need `A` **Schur-stable** (spectral radius `< 1`) so the infinite-horizon sum converges; guard
  and document — an unstable `A` makes the Lyapunov solution meaningless.
- Reuse `GaussianElimination` (factored form) for the rank pivots rather than forming any inverse.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/control_analysis/ControllabilityObservability.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on the matrix builds, and
  `extern template class ControllabilityObservability<float, 2, 1, 1>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/control_analysis/ControllabilityObservability.cpp` →
  `template class ControllabilityObservability<float, 2, 1, 1>;`
- Test: `numerical/control_analysis/test/TestControllabilityObservability.cpp`
- Doc: `doc/control_analysis/ControllabilityObservability.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestControllabilityObservability.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
