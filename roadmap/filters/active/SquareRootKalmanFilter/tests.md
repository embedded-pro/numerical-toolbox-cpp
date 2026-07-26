# Square-Root / Information Kalman Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSquareRootKalmanFilter : public ::testing::Test:
    # 2-state constant-velocity model, scalar position measurement
    SquareRootKalmanFilter<float, 2, 1> filter{ x0, cholesky(P0) }
    # reference for cross-checks:
    KalmanFilter<float, 2, 1>           reference{ x0, P0 }
# each case below is a TEST_F(TestSquareRootKalmanFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
matches_conventional_kalman:
    Arrange: identical F, H, Q, R and measurement stream fed to both filters
    Act:     Predict + Update each step
    Assert:  State() and Covariance() ≈ reference within tight tolerance

covariance_stays_positive_definite:
    Arrange: run many predict/update cycles
    Assert:  cholesky(Covariance()) succeeds every step (all pivots > 0)

factor_reconstructs_covariance:
    Arrange: after several updates
    Assert:  CovarianceFactor() * CovarianceFactorᵀ ≈ Covariance()

predict_grows_uncertainty:
    Arrange: call Predict() with no measurement
    Assert:  trace(Covariance()) increases by ≈ trace(Q)

update_shrinks_uncertainty:
    Arrange: Predict then Update(z)
    Assert:  trace(Covariance()) decreases

ill_conditioned_stays_stable:
    Arrange: P0 with cond ≈ 1e8 (a conventional KF would lose PSD)
    Assert:  filter remains PSD and finite (no NaN), tracks truth

perfect_measurement_collapses_variance:
    Arrange: R -> 0 on the measured axis
    Assert:  posterior variance of that axis -> ~0, no divide blow-up

vague_measurement_is_ignored:
    Arrange: R very large
    Assert:  State() ≈ prediction (gain -> 0)

steady_state_gain_converges:
    Arrange: time-invariant system, iterate
    Assert:  gain and Covariance() converge to fixed values

reset_and_getters:
    Assert:  State()/CovarianceFactor() return the constructor-seeded values
```

## Reference vectors

- Cross-checked against `KalmanFilter` (item 39 reuses `KalmanFilterBase`) on a 2-state CV model.
- Scalar steady-state: converged variance matches the algebraic Riccati fixed point.

## Edge cases

- `Q = 0` (no process noise) ⇒ covariance only shrinks, factor stays lower-triangular.
- `R -> 0` (perfect sensor) and `R -> ∞` (useless sensor) ⇒ well-defined limits, no blow-up.
- 1-state scalar filter ⇒ QR reduces to a single Givens rotation; matches closed form.
- Near-singular `P0` ⇒ square-root path keeps eigenvalues ≥ 0 where the conventional form fails.
