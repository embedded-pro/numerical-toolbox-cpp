# Residual-Form Kalman Update — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestResidualKalmanUpdate : public ::testing::Test:
    Vector<float, 2> x{ 1.0, 0.5 }
    SquareMatrix<float, 2> P{ { 0.5, 0.1 }, { 0.1, 0.3 } }
    Matrix<float, 1, 2> H{ { 1, 0 } };  SquareMatrix<float, 1> R{ 0.2 }
# each case below is a TEST_F(TestResidualKalmanUpdate, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
scalar_update_matches_the_kalman_equations:
    Act:    Apply(x, P, r = (0.4), H, R)                        # z = 1.4
    Assert: status applied, nis ≈ 0.228571;  x ≈ (1.285714, 0.557143);  P ≈ [[0.142857, 0.028571], [0.028571, 0.285714]]
vector_update_matches_the_kalman_equations:
    Arrange: H₂ = [[1, 0], [1, 1]], R₂ = diag(0.2, 0.1), r = (1.4, 1.2) − H₂x = (0.4, −0.3)
    Assert:  nis ≈ 0.934146;  x ≈ (1.097561, 0.212195);  P ≈ [[0.092683, −0.063415], [−0.063415, 0.117073]]
nis_matches_the_applied_update:
    Assert: Nis((0.4), H, P, R) ≈ 0.228571 == Apply(...).nis
gate_rejects_an_outlier_without_touching_the_state:
    Act:    Apply(x, P, r = (3.0), H, R, 6.63)                  # z = 4.0
    Assert: status gated, nis ≈ 12.857143;  x and P unchanged
singular_innovation_covariance_is_reported:
    Arrange: P = 0, R = 0
    Assert:  status singular;  x and P unchanged;  Nis(...) == nullopt
wrapped_residual_keeps_an_angle_estimate_sane:
    Arrange: x = (3.1), P = (0.04), R = (0.01), z = −3.1
    Act:     Apply with r = AngleDifference(z, 3.1) = 0.083185, then WrapToPi(x)
    Assert:  x ≈ −3.116637;  P ≈ 0.008;  nis ≈ 0.138396    # the raw residual −6.2 would drive x to −1.86
joseph_form_stays_symmetric:
    Assert: after 100 scalar updates of the fixture, P(0,1) == P(1,0) and both eigenvalues > 0
```

## Reference vectors

- Fixture: `S = 0.7`, `K = (0.714286, 0.142857)`.
- 2-D: `S = [[0.7, 0.6], [0.6, 1.1]]`, `NIS = 0.934146`.
- Angle: residual `0.083185` (wrapped) against `−6.2` (raw).

## Edge cases

- `gate = ∞` (default) never gates.
- `nis` exactly at the gate is applied (`>` test).
