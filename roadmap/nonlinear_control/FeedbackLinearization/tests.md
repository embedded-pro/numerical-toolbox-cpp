# Feedback Linearization — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestFeedbackLinearization : public ::testing::Test:
    # Injected control-affine model is mocked so the law is verified in isolation:
    StrictMock<nonlinear_control::MockControlAffineModel<float,2>> model
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    FeedbackLinearization<float,2> controller{ model, Kp, Kd }
# each case below is a TEST_F(TestFeedbackLinearization, <name>)
```

## Test cases (Arrange / Act / Assert)

```
cancels_to_integrator_chain:
    Arrange: model returns B=I, a=0; error and error-rate = 0, ydDdot = c
    Act:     u = ComputeInput(...)
    Assert:  u == c           (u = B·v reduces to the commanded virtual input)

adds_drift_compensation:
    Arrange: model a(x) = [0, d]; all setpoints match state, v = 0
    Assert:  u == a(x)        (pure drift-cancelling input)

pd_law_drives_position_error:
    Arrange: e = yd - x != 0, eDot = 0, feedforward = 0, B=I
    Assert:  u == Kp · e      (proportional term appears through B·v)

pd_law_drives_velocity_error:
    Arrange: eDot != 0, e = 0, B=I
    Assert:  u == Kd · eDot

decoupling_matrix_scales_virtual_input:
    Arrange: B = diag(2,3), v = [1,1]
    Assert:  u == [2,3]       (B·v applied, not v alone)

closed_loop_error_decays:
    Arrange: wrap a plant that integrates ÿ = B⁻¹(u − a); run K steps
    Assert:  ||yd − x|| -> 0 monotonically (exact linearization)

reference_feedforward_used:
    Arrange: ydDdot = c, all errors 0, B=I
    Assert:  u == c           (feedforward passes through)
```

## Reference vectors

- Ideal cancellation: with the exact model, closed-loop error obeys `ë + Kd·ė + Kp·e = 0` — a
  linear ODE whose decay rate is hand-computable from `Kp`, `Kd`.
- Computed-torque instance (single pendulum): with `B = M`, `a = g(q)`, the gravity-hold input at
  zero acceleration is the golden `τ = m g L sin(q)`.

## Edge cases

- Model mismatch (mock `B` scaled by 1.2): closed loop stays stable but shows bounded steady error.
- `ydDdot` large: input must not saturate the (documented) actuator model in tests.
- Near-singular `B` / decoupling matrix in the general input-output form: division guard exercised.
