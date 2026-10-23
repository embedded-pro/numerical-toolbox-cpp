# Tridiagonal Solver (Thomas Algorithm) — Implementation Pseudocode

> Roadmap ref: #59 (Tier 1) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

`O(n)` solve of `A·x = b` for a tridiagonal `A`, factored once and reused for many right-hand sides.
Cubic splines (#61), implicit finite-difference schemes and some smoothing filters all reduce to it; a
dense LU on the same system costs `O(n³)`.

## Data structures

```cpp
template<typename T, std::size_t MaxSize>      # static_assert(std::is_floating_point_v<T>); static_assert(MaxSize >= 1)
class TridiagonalSolver:                       # instantiated for float
    using Band = std::array<T, MaxSize>
    Band sub                                   # a_k: A[k][k−1], a_0 unused
    Band invPivot                              # 1 / (b_k − a_k·c'_{k−1})
    Band supPrime                              # c'_k = c_k · invPivot_k
    std::size_t size                           # n, 1 ≤ n ≤ MaxSize; 0 until Factor succeeds
```

## Interface

```cpp
bool Factor(const Band& sub, const Band& diag, const Band& sup, std::size_t n)   # false if n ∉ [1, MaxSize] or a pivot vanishes
Band Solve(const Band& rhs) const              # hot path; entries ≥ n are zero; precondition: Factor() == true
std::size_t Size() const
```

## Algorithm (pseudocode)

```text
function Factor(a, b, c, n):                   # forward elimination of the matrix only
    if n == 0 or n > MaxSize: return false
    scale = max |b_k|, |a_k|, |c_k| over the n rows
    pivot = b_0;  if |pivot| ≤ ε·scale: return false
    invPivot[0] = 1/pivot;  supPrime[0] = c_0·invPivot[0]
    for k = 1..n−1:
        pivot = b_k − a_k·supPrime[k−1]
        if |pivot| ≤ ε·scale: return false
        invPivot[k] = 1/pivot;  supPrime[k] = c_k·invPivot[k]
    store a as sub;  size = n;  return true

function Solve(r):                             # OPTIMIZE_FOR_SPEED — O(n)
    y[0] = r_0·invPivot[0]
    for k = 1..n−1:          y[k] = (r_k − sub[k]·y[k−1])·invPivot[k]
    x[n−1] = y[n−1]
    for k = n−2 down to 0:   x[k] = y[k] − supPrime[k]·x[k+1]
    return x
```

## Complexity & memory

- `Factor`: `O(n)`, `n` divisions; `Solve`: `O(n)`, `3n` multiply-adds, no division.
- Memory: three `MaxSize` bands; no heap.

## Numerical / embedded notes

- **No pivoting:** stable for strictly diagonally dominant or symmetric positive-definite `A` (the spline
  systems are both). For other tridiagonal matrices use `LuDecomposition` with partial pivoting.
- The `ε·scale` pivot test reports a singular or non-dominant system as `false` instead of dividing by
  (near) zero.
- Factoring once moves every division out of `Solve` — useful when the matrix depends only on knot
  spacing and the right-hand side changes per channel or per tick.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/math/TridiagonalSolver.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class TridiagonalSolver<float, 8>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/TridiagonalSolver.cpp` → `template class TridiagonalSolver<float, 8>;`
- Test: `numerical/math/test/TestTridiagonalSolver.cpp`
- Doc: `doc/math/TridiagonalSolver.md` (per `doc/TEMPLATE.md`) + row in `doc/math/README.md`
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestTridiagonalSolver.cpp` → `numerical.math_test`.
  It lives in `math` next to `TriangularSolve` because `CubicSpline` (#61, also `math`) uses it and
  `numerical.solver` already depends on `numerical.math`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
