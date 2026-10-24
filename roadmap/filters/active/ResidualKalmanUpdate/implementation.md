# Residual-Form Kalman Update with χ² Gating — Implementation Pseudocode

> Roadmap ref: #74 (Tier 2) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

A stateless Kalman measurement update for estimators that own their state and must form the
innovation themselves. Typical cases are angles that must be wrapped (bearings, headings), a measurement
model chosen per observation (one landmark among many), and measurements that must be gated or
associated by Mahalanobis distance before they are applied. The caller passes the residual `r = z ⊖ h(x)`,
the Jacobian `H` and the noise `R`. The update returns the normalised innovation squared (NIS) and either
applies the Joseph-form update or rejects it at the gate. `ExtendedKalmanFilter` cannot serve these
cases: it forms `z − h(x)` itself, fixes `h` at construction, and hides `S`.

## Data structures

```cpp
enum class KalmanUpdateStatus : std::uint8_t { applied, gated, singular }

template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct KalmanUpdateResult:
    KalmanUpdateStatus status
    T                  nis                      # rᵀ S⁻¹ r (0 when singular)

template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
class ResidualKalmanUpdate                      # stateless; static members only
    using StateVector = math::Vector<T, StateSize>;  using Covariance = math::SquareMatrix<T, StateSize>
    using Residual = math::Vector<T, MeasurementSize>;  using Jacobian = math::Matrix<T, MeasurementSize, StateSize>
    using MeasurementNoise = math::SquareMatrix<T, MeasurementSize>
```

## Interface

```cpp
static std::optional<T> Nis(const Residual& r, const Jacobian& H, const Covariance& P, const MeasurementNoise& R)
static KalmanUpdateResult<T> Apply(StateVector& x, Covariance& P, const Residual& r, const Jacobian& H,
                                   const MeasurementNoise& R, T gate = std::numeric_limits<T>::infinity())   # hot path
```

`Nis` lets a caller score several candidate associations before committing one with `Apply`. The caller
normalises the state afterwards (for example re-wraps a heading), because only it knows which components
are angles.

## Algorithm (pseudocode)

```text
function Nis(r, H, P, R):
    S = CongruenceTransform(H, P) + R            # H·P·Hᵀ + R; upstream MatrixOperations
    return ConsistencyMetrics<T, M>::Nis(r, S)   # Cholesky solve; nullopt if S is not SPD

function Apply(x, P, r, H, R, gate):             # OPTIMIZE_FOR_SPEED
    S = CongruenceTransform(H, P) + R
    L = CholeskyDecomposition<T, M>::TryFactor(S)
    if L is nullopt: return { singular, 0 }      # x, P untouched
    y = L-solve(S, r);  nis = r·y                # S⁻¹r via the factor
    if nis > gate: return { gated, nis }         # x, P untouched
    PHt = P·Hᵀ
    K = (L-solve(S, PHtᵀ))ᵀ                      # K = P·Hᵀ·S⁻¹, M triangular-solve pairs, no explicit inverse
    x += K·r
    A = I − K·H
    P = Symmetrize(A·P·Aᵀ + K·R·Kᵀ)              # Joseph form: stays symmetric PSD in float
    return { applied, nis }
```

## Complexity & memory

- `O(N²·M + N·M² + M³)`: one `M×M` Cholesky and `N`-by-`M` products. For the localization cases
  (`N ≤ 4`, `M ≤ 2`) that is under 200 multiply-adds.
- Stack only: `S`, its factor, `K`, `A`. No heap.

## Numerical / embedded notes

- **Gate values.** Gate against the χ² quantile for `M` degrees of freedom: 6.63 (M = 1) and 9.21 (M = 2)
  at 99 %, 3.84 / 5.99 at 95 %. `ConsistencyMetrics` holds the two-sided 95 % bounds for NEES/NIS
  monitoring; gating uses only the upper tail.
- **Joseph form.** `(I − KH)P(I − KH)ᵀ + KRKᵀ` is valid for any gain and stays PSD under `float` rounding,
  unlike `(I − KH)P`. `Symmetrize` removes the asymmetry the products leave.
- **Residual convention.** The residual is `z ⊖ h(x)`: for an angle component, the caller wraps it with
  `AngleDifference` (item 54). Passing an unwrapped `±2π` jump is the classic bearing-EKF divergence, and
  the test shows it.
- **Sequential updates.** Independent measurements (several landmarks) can be applied one after
  another, each with its own `H` and `R`; for a linear model that equals one stacked update.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/filters/active/ResidualKalmanUpdate.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Apply`, and `extern template class ResidualKalmanUpdate<float, 3, 2>;` / `<float, 4, 1>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/active/ResidualKalmanUpdate.cpp` → the same instantiations.
- Test: `numerical/filters/active/test/TestResidualKalmanUpdate.cpp`
- Doc: `doc/filters/active/ResidualKalmanUpdate.md` (per `doc/TEMPLATE.md`) + row in `doc/filters/README.md`
- CMake: `.hpp` → `target_sources(numerical.filters.active PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.filters.active ...)`; test → `numerical.filters.active_test`.
- Reuses: `CholeskyDecomposition`, `ConsistencyMetrics`, `MatrixOperations` (`CongruenceTransform`, `Symmetrize`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
