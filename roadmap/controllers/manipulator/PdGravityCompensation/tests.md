# PD + Gravity Compensation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPdGravityCompensation : public ::testing::Test:
    # Injected model mocked so torque math is verified in isolation:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    PdGravityCompensation<float,2> controller{ model, Kp, Kd }
# each case below is a TEST_F(TestPdGravityCompensation, <name>)
```

## Test cases (Arrange / Act / Assert)

```
gravity_only_at_matched_setpoint:
    Arrange: q = qd, qDot = 0, model g(q) = [0, mgL]
    Act:     τ = ComputeTorque(q, 0, qd)
    Assert:  τ == g          (pure holding torque, no PD contribution)

proportional_term_on_position_error:
    Arrange: e = qd - q != 0, qDot = 0, g = 0
    Assert:  τ == Kp · e

derivative_term_damps_velocity:
    Arrange: qDot != 0, e = 0, g = 0
    Assert:  τ == -Kd · qDot

superposition_of_all_terms:
    Arrange: e != 0, qDot != 0, g != 0
    Assert:  τ == Kp·e - Kd·qDot + g

diagonal_gains_decouple_joints:
    Arrange: error only in joint 0, g = 0
    Assert:  torque appears only in joint 0

zero_error_zero_velocity_holds_gravity:
    Arrange: e = 0, qDot = 0
    Assert:  τ == g          (equilibrium holding torque)

only_gravity_method_queried:
    Arrange: any state
    Assert:  ComputeGravityTerms called once; ComputeMassMatrix / ComputeCoriolisTerms
             never called (StrictMock enforces)

regulates_to_setpoint_closed_loop:
    Arrange: wrap plant q̈ = M⁻¹(τ − Cq̇ − g); run K steps from q0 != qd
    Assert:  q -> qd, ||e|| -> 0 with zero steady-state error
```

## Reference vectors

- Single-pendulum hold: `g(q) = m g L sin(q)`; at `qd` with `e = 0`, `q̇ = 0` ⇒ `τ = m g L sin(qd)`.
- Unique closed-loop equilibrium at `e = 0` (SPD `Kp` invertible), independent of `Kd`.

## Edge cases

- Large initial error: `τ` bounded by `Kp·e_max + g_max` (external actuator clamp documented).
- `Kp → 0`: convergence slows but stays stable while `Kp` remains SPD.
- Gravity model mismatch (mock `g` scaled 1.1): bounded steady droop `e_ss = Kp⁻¹·Δg`.
