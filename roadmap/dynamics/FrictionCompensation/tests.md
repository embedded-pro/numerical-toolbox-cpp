# Friction Compensation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestFrictionCompensation : public ::testing::Test:
    # single-joint params: Fc=0.5, Fs=0.8, Fv=0.1, vs=0.05, delta=2, eps=1e-3
    JointFrictionParameters<float> p = MakeParams()
    FrictionCompensation<float, 1> friction{ {p} }
# each case below is a TEST_F(TestFrictionCompensation, <name>)
```

## Test cases (Arrange / Act / Assert)

```
positive_velocity_positive_torque:
    Arrange: qDot = +1.0
    Act:     tau = Compute(qDot)
    Assert:  tau[0] > 0 and tau[0] ≈ Fc + Fv*1.0 (dip negligible at high v)

sign_flips_with_velocity:
    Arrange: compare Compute(+v) and Compute(-v)
    Assert:  tau(+v) ≈ -tau(-v)  (odd symmetry)

viscous_dominates_at_high_speed:
    Arrange: sweep large |qDot|
    Assert:  tau grows ~linearly with slope Fv

stiction_exceeds_coulomb_near_zero:
    Arrange: small v just above eps
    Assert:  |level| closer to Fs than Fc  (Stribeck peak)

stribeck_decays_to_coulomb:
    Arrange: v >> v_s
    Assert:  level -> Fc within tol

smooth_through_zero:
    Arrange: v = 0
    Assert:  tau[0] == 0 (tanh(0)=0), continuous, no jump

multi_joint_elementwise:
    Arrange: FrictionCompensation<float,3>, distinct params, qDot vector
    Assert:  each output equals its per-joint JointTorque

feedforward_additive_cancels_model:
    Arrange: plant friction = model; command = -Compute(qDot)
    Assert:  net joint friction ≈ 0 (compensation cancels)
```

## Reference vectors

- High speed `v=1`, dip≈0: `tau ≈ Fc·tanh(1/eps) + Fv·1 ≈ Fc + Fv`.
- `v=0`: `tau = 0` exactly (odd, smoothed).
- `v = v_s`: `fall = e^{-1}`, `level = Fc + (Fs−Fc)/e`.

## Edge cases

- `qDot = 0` — no torque, no NaN from the `^delta` power.
- Very small `eps` — steep but finite slope; assert no overflow.
- `Fs == Fc` — Stribeck term vanishes, pure Coulomb+viscous.
- Negative-going zero crossing — output stays continuous (no chatter).
