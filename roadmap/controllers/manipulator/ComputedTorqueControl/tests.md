# Computed-Torque Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestComputedTorqueControl : public ::testing::Test:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    ComputedTorqueControl<float,2> controller{ model, Kp, Kd }
# each case below is a TEST_F(TestComputedTorqueControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
pure_feedforward_tracks_acceleration:
    Arrange: model M=I, C=0, g=0; all errors 0, qdDdot = a
    Act:     τ = ComputeTorque(...)
    Assert:  τ == a           (M·aq reduces to the commanded acceleration)

gravity_compensation_at_rest:
    Arrange: model g(q) = [0, mgL]; all setpoints match state, aq = 0
    Assert:  τ == g

coriolis_terms_added:
    Arrange: model C(q,q̇)q̇ = c; aq = 0
    Assert:  τ == c

mass_matrix_shapes_command:
    Arrange: M = diag(2,3), aq = [1,1] (via qdDdot, errors 0)
    Assert:  τ == [2,3]

position_error_maps_through_mass:
    Arrange: e = qd - q != 0, others 0, M = I
    Assert:  τ == Kp · e

velocity_error_maps_through_mass:
    Arrange: eDot != 0, e = 0, M = I
    Assert:  τ == Kd · eDot

full_law_superposition:
    Arrange: nonzero M, C, g, and errors
    Assert:  τ == M·(qdDdot + Kd·ė + Kp·e) + Cq̇ + g

decoupled_error_dynamics:
    Arrange: wrap plant q̈ = M⁻¹(τ − Cq̇ − g); run K steps
    Assert:  ||qd − q|| -> 0 matching ë + Kd·ė + Kp·e = 0

all_three_model_terms_queried:
    Arrange: any state
    Assert:  ComputeMassMatrix, ComputeCoriolisTerms, ComputeGravityTerms each called once
             (StrictMock)
```

## Reference vectors

- With the exact model, closed-loop error obeys `ë + Kd·ė + Kp·e = 0` — a linear ODE whose decay
  rate is hand-computable from `Kp`, `Kd`.
- 2-link at rest with all setpoints matched: golden `τ = g(q)`.

## Edge cases

- Near-singular `M` (mock): still multiplied, never inverted ⇒ no torque blow-up.
- Model mismatch (mock `M` scaled 1.2): bounded tracking error, closed loop stays stable.
- Large `qdDdot`: torque stays within the documented actuator model.
