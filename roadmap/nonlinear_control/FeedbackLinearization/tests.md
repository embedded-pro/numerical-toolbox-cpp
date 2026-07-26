# Feedback Linearization — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestFeedbackLinearization : public ::testing::Test:
    # Injected nonlinear model is mocked so torque math is verified in isolation:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    FeedbackLinearization<float,2> controller{ model, Kp, Kd }
# each case below is a TEST_F(TestFeedbackLinearization, <name>)
```

## Test cases (Arrange / Act / Assert)

```
cancels_to_double_integrator:
    Arrange: model returns M=I, C=0, g=0; error and error-rate = 0, qdDdot = a
    Act:     τ = ComputeTorque(...)
    Assert:  τ == a           (τ = M·v reduces to the commanded acceleration)

adds_gravity_compensation:
    Arrange: model g(q) = [0, mgL]; all setpoints match state, v = 0
    Assert:  τ == g(q)        (pure gravity hold torque)

adds_coriolis_terms:
    Arrange: model C(q,q̇)q̇ = c; v = 0
    Assert:  τ == c

pd_law_drives_position_error:
    Arrange: e = qd - q != 0, eDot = 0, feedforward = 0, M=I
    Assert:  τ == Kp · e      (proportional term appears through M·v)

pd_law_drives_velocity_error:
    Arrange: eDot != 0, e = 0, M=I
    Assert:  τ == Kd · eDot

mass_matrix_scales_virtual_input:
    Arrange: M = diag(2,3), v = [1,1]
    Assert:  τ == [2,3]       (M·v applied, not v alone)

closed_loop_error_decays:
    Arrange: wrap a plant that integrates q̈ = M⁻¹(τ − Cq̇ − g); run K steps
    Assert:  ||qd − q|| -> 0 monotonically (exact linearization)

reference_feedforward_used:
    Arrange: qdDdot = a, all errors 0, M=I
    Assert:  τ == a           (acceleration feedforward passes through)
```

## Reference vectors

- Ideal cancellation: with the exact model, closed-loop error obeys `ë + Kd·ė + Kp·e = 0` — a
  linear ODE whose decay rate is hand-computable from `Kp`, `Kd`.
- Single pendulum gravity hold: `τ = m g L sin(q)` is the golden torque at zero acceleration.

## Edge cases

- Model mismatch (mock M scaled by 1.2): closed loop stays stable but shows bounded steady error.
- `qdDdot` large: torque must not saturate the (documented) actuator model in tests.
- Near-singular `M` in the general input-output form: division guard exercised.
