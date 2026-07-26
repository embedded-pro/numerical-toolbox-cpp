# Momentum-Based Collision Observer — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class MockDynamics : public Dynamics<float, 2>:
    MOCK_METHOD((Matrix<float,2,2>), MassMatrix, (q), (const, override))
    MOCK_METHOD((Vector<float,2>), CoriolisTranspose, (q, qd), (const, override))
    MOCK_METHOD((Vector<float,2>), Gravity, (q), (const, override))

class TestMomentumObserver : public ::testing::Test:
    StrictMock<MockDynamics> model
    Vector<2> gain = {50, 50}
    MomentumObserver<float, 2> observer{ model, gain }
# each case below is a TEST_F(TestMomentumObserver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
no_external_torque_gives_zero_residual:
    Arrange: consistent M,C,g; tau equals internal dynamics torque; no contact
    Act:     Reset then Update over steps
    Assert:  residual ≈ 0

step_external_torque_is_tracked:
    Arrange: inject constant τ_ext on joint 0
    Act:     Update repeatedly
    Assert:  r[0] rises toward τ_ext (first-order); r[1] stays small

higher_gain_converges_faster:
    Arrange: two observers K_O = 20 vs 200, same τ_ext
    Assert:  larger gain reaches τ_ext in fewer steps

reset_captures_initial_momentum:
    Arrange: nonzero q̇ at reset
    Assert:  first residual after Reset ≈ 0 (p0 subtracted)

collision_detected_above_threshold:
    Arrange: residual driven past threshold
    Assert:  CollisionDetected(threshold) == true; false below

residual_decays_after_contact_release:
    Arrange: τ_ext applied then removed
    Assert:  r returns toward 0 with the observer time constant

per_joint_localization:
    Arrange: contact only on joint 1
    Assert:  |r[1]| >> |r[0]|
```

## Reference vectors

- Static arm (q̇ = 0), gravity-consistent τ ⇒ residual 0 at every step.
- First-order response: with gain `k`, `r(t) ≈ τ_ext·(1 − e^{−k·t})` for a constant external torque.

## Edge cases

- Large `dt` step ⇒ integration drift; assert bounded residual under model bias.
- Zero gain ⇒ residual stays 0 (observer disabled).
- Model bias in `g(q)` ⇒ constant residual offset (documents the Reset-to-rezero behavior).
