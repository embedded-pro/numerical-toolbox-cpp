# Square-Root / Information Kalman Filter — Implementation Pseudocode

> Roadmap ref: #39 (Tier 4) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class SquareRootKalmanFilter:
    Vector<T, StateSize>                     x        # state estimate
    SquareMatrix<T, StateSize>               S        # Cholesky factor: P = S * Sᵀ (lower-tri)
    SquareMatrix<T, StateSize>               sqrtQ    # process-noise factor
    SquareMatrix<T, MeasurementSize>         sqrtR    # measurement-noise factor
    Matrix<T, StateSize, StateSize>          F        # state transition
    Matrix<T, MeasurementSize, StateSize>    H        # measurement model
    # (control B when ControlSize > 0)
```

## Interface

```
SquareRootKalmanFilter(Vector x0, SquareMatrix S0)     # S0 = chol(P0)
void SetStateTransition(F)      / SetMeasurementMatrix(H)
void SetProcessNoiseFactor(sqrtQ) / SetMeasurementNoiseFactor(sqrtR)
void Predict()  /  Predict(Vector u)    requires ControlSize > 0
void Update(Vector z)
Vector       State() const
SquareMatrix Covariance() const           # reconstruct S * Sᵀ on demand
SquareMatrix CovarianceFactor() const     # return S directly
```

## Algorithm (pseudocode)

```
function Predict():                              # OPTIMIZE_FOR_SPEED  (time update via QR)
    x = F * x  (+ B * u)
    # Triangularize the pre-array so P⁻ = S⁻ S⁻ᵀ = F P Fᵀ + Q, never forming P explicitly:
    A       = [ (F * S)ᵀ ;  sqrtQᵀ ]             # (2n x n) stacked block
    R       = QrTriangularize(A)                 # Householder/Givens -> upper-tri R (item 27)
    S       = Rᵀ                                 # new lower-tri covariance factor

function Update(z):                              # OPTIMIZE_FOR_SPEED  (measurement update via QR)
    # Pre-array mixes measurement and state factors:
    #        [ sqrtR      H * S ]        QR        [ Sy      K̃ ]
    #        [   0          S   ]   ───────────>   [  0      S⁺ ]
    Pre     = [ [ sqrtR , H*S ] ; [ 0 , S ] ]
    Post    = QrTriangularize(Pre)
    Sy      = Post.topLeft(MeasurementSize)      # innovation-covariance factor
    Ktilde  = Post.topRight()                    # cross block
    S       = Post.bottomRight(StateSize)        # updated factor, guaranteed lower-tri / PSD
    innov   = z - H * x
    K       = Ktilde * Inverse(Sy)               # gain via triangular solve (Sy is triangular)
    x       = x + K * innov
```

## Complexity & memory

- Time: `O((n+m)·n²)` per step, dominated by the QR triangularization (`n = StateSize`, `m = MeasurementSize`).
- Memory: `O(n²)` — all factors and pre-arrays are fixed-size `std::array`-backed `math::Matrix`; no heap.

## Numerical / embedded notes

- Propagating a **factor** `S` (never `P`) guarantees `P = S Sᵀ` stays symmetric positive-definite
  even in reduced precision — the failure mode that makes a conventional KF "blow up" cannot occur.
- The **effective condition number is halved**: `cond(S) = sqrt(cond(P))`, so more state bits survive.
- Build QR from **Givens rotations** (structured, in-place) or Householder (item 27); both avoid the
  squaring of dynamic range inherent in forming `F P Fᵀ`.
- **Information form** is the dual: propagate a factor of `P⁻¹` instead, which is cheaper when many
  measurements are fused per step and lets an "uninitialized" state be encoded as zero information.
- Reuses `KalmanFilterBase` (state/noise storage), `CholeskyDecomposition`, and QR (item 27);
  `Predict`/`Update` carry `OPTIMIZE_FOR_SPEED`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/active/SquareRootKalmanFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Predict`/`Update`, and
  `extern template class SquareRootKalmanFilter<float, StateSize, MeasurementSize>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/active/SquareRootKalmanFilter.cpp` →
  `template class SquareRootKalmanFilter<float, StateSize, MeasurementSize>;`
- Test: `numerical/filters/active/test/TestSquareRootKalmanFilter.cpp`
- Doc: `doc/filters/active/SquareRootKalmanFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestSquareRootKalmanFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
