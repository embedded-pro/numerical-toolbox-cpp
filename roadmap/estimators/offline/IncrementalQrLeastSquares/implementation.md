# Incremental QR Least Squares (rank-revealing) — Implementation Pseudocode

> Roadmap ref: #63 (Tier 3) · Target: `numerical/estimators/offline` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

Batch linear least squares `min ‖Y·π − τ‖` over an unbounded number of rows with **fixed memory**: each
row is rotated into an upper-triangular factor with Givens rotations as it arrives, no sample is stored,
and `YᵀY` is never formed. The solve is rank-revealing (SVD of the `P×P` factor), returning the
minimum-norm solution and the identifiable parameter combinations when the regressor is rank-deficient.
Consumers: robot inertial-parameter identification (robotics-toolbox-cpp), motor/drive parameter
identification, sensor calibration from long logs.

## Data structures

```cpp
template<typename T, std::size_t Parameters>          # static_assert(std::is_floating_point_v<T>); instantiated for float
struct LeastSquaresResult:
    math::Vector<T, Parameters>             parameters    # minimum-norm solution
    std::size_t                             rank          # number of identifiable combinations
    math::Matrix<T, Parameters, Parameters> rowSpace      # first `rank` rows span the identifiable combinations
    T                                       residualRms   # sqrt(‖Y·π̂ − τ‖² / rows)

template<typename T, std::size_t Parameters>
class IncrementalQrLeastSquares:
    math::SquareMatrix<T, Parameters> R                    # upper-triangular factor (STATE)
    math::Vector<T, Parameters>       d                    # Qᵀ·τ (STATE)
    T                                 residualSquares      # part of ‖τ‖² outside range(R) (STATE)
    std::size_t                       rows
```

## Interface

```cpp
void AddRow(const math::Vector<T, Parameters>& row, T rhs)                 # O(P²); hot path of the accumulation
template<std::size_t Rows>
void AddRows(const math::Matrix<T, Rows, Parameters>& block, const math::Vector<T, Rows>& rhs)   # AddRow per row
std::optional<LeastSquaresResult<T, Parameters>> Solve(T relativeTolerance) const   # nullopt if no rows
void Reset()
std::size_t Rows() const
```

## Algorithm (pseudocode)

```text
function AddRow(row, rhs):                     # OPTIMIZE_FOR_SPEED — streaming Givens QR
    for j in 0..P−1:                           # rotate the row into R
        if row[j] == 0: continue
        g = math::ComputeGivens(R[j][j], row[j])
        rotate (R[j][j..P−1], d[j]) with (row[j..P−1], rhs) using g     # zeroes row[j]
    residualSquares += rhs²                    # what is left lies outside range(R)
    rows += 1

function Solve(tol):
    if rows == 0: return nullopt
    svd.Decompose(R)                           # SingularValueDecomposition<T, P, P>
    rank = #{ σᵢ > tol·σ₀ }
    π̂ = Σ_{i<rank} vᵢ·(uᵢᵀ·d)/σᵢ                # minimum-norm least squares
    rowSpace = Vᵀ (rows ≥ rank are the unidentifiable directions)
    residualRms = sqrt((residualSquares + Σ_{i≥rank}(uᵢᵀ·d)²) / rows)
    return { π̂, rank, rowSpace, residualRms }
```

## Complexity & memory

- `AddRow`: `O(P²)` (at most `P` rotations of length `≤ P`); `Solve`: one `P×P` SVD.
- Memory: `P² + P + 2` scalars regardless of the number of rows (e.g. `P = 60`: ≈ 14 KB in `float`); no heap.
  An offline / commissioning routine, not an ISR path — but accumulation can run at the sample rate.

## Numerical / embedded notes

- Streaming QR avoids forming `YᵀY`, which squares the condition number — the reason to prefer it over
  normal equations in `float`.
- Row order does not change the result beyond rounding (orthogonal transformations).
- Unlike `RecursiveLeastSquares` (exponential forgetting, `P×P` covariance update) this weighs all rows
  equally and reveals rank; unlike `LinearRegression` it never stores the samples.
- Column scaling matters for the rank decision: scale columns to comparable magnitude before accumulating
  when parameters have different units.
- Weighted least squares: multiply a row and its right-hand side by `√w` before `AddRow`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/estimators/offline/IncrementalQrLeastSquares.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED`
  on `AddRow`, and `extern template class IncrementalQrLeastSquares<float, 2>;` / `<float, 3>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/offline/IncrementalQrLeastSquares.cpp` → the same instantiations.
- Test: `numerical/estimators/offline/test/TestIncrementalQrLeastSquares.cpp`
- Doc: `doc/estimators/IncrementalQrLeastSquares.md` (per `doc/TEMPLATE.md`) + row in `doc/estimators/README.md`
- CMake: `.hpp` → `target_sources(numerical.estimators.offline PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.estimators.offline ...)`; test → `numerical.estimators.offline_test`.
- Depends on: `math::GivensRotation`, `solvers::SingularValueDecomposition`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
