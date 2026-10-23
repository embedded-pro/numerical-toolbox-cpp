# Friction Compensation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestFrictionCompensation : public ::testing::Test:
    # single-axis params: Fc=0.5, Fs=0.8, Fv=0.1, vs=0.05, delta=2, eps=1e-3, maxCompensation=10
    FrictionParameters<float> p = MakeParams()
    FrictionCompensation<float, 1> friction{ {p} }
# each case below is a TEST_F(TestFrictionCompensation, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
positive_velocity_positive_torque:
    Arrange: velocity = +1.0
    Act:     tau = Compute(velocity)
    Assert:  tau[0] > 0 and tau[0] ≈ Fc + Fv*1.0 (dip negligible at high v)

sign_flips_with_velocity:
    Arrange: compare Compute(+v) and Compute(-v)
    Assert:  tau(+v) ≈ -tau(-v)  (odd symmetry)

viscous_dominates_at_high_speed:
    Arrange: sweep large |velocity|
    Assert:  tau grows ~linearly with slope Fv

stiction_exceeds_coulomb_near_zero:
    Arrange: v = 10·eps (tanh ≈ 1) and v << v_s
    Assert:  AxisTorque(p, v) − Fv·v ≈ Fs  (Stribeck peak)

stribeck_decays_to_coulomb:
    Arrange: v >> v_s
    Assert:  AxisTorque(p, v) − Fv·v → Fc within tol

smooth_through_zero:
    Arrange: v = 0
    Assert:  tau[0] == 0 (tanh(0)=0), continuous, no jump

multi_axis_elementwise:
    Arrange: FrictionCompensation<float,3>, distinct params, velocity vector
    Assert:  each output equals its per-axis AxisTorque

feedforward_additive_cancels_model:
    Arrange: plant friction torque = −model(v) (opposes motion); command = τ_control + Compute(v)
    Assert:  net torque reaching the load ≈ τ_control (compensation cancels)
output_is_clamped_to_max_compensation:
    Arrange: maxCompensation = 0.6, v large
    Assert:  |Compute(v)| == 0.6
```

## Reference vectors

- High speed `v=1`, dip≈0: `tau ≈ Fc·tanh(1/eps) + Fv·1 ≈ Fc + Fv = 0.6` (fixture `maxCompensation = 10`).
- `v=0`: `tau = 0` exactly (odd, smoothed).
- `v = v_s`: `fall = e^{-1}`, `level = Fc + (Fs−Fc)/e`.

## Edge cases

- `velocity = 0` — no torque, no NaN from the `^delta` power.
- Very small `eps` — steep but finite slope; assert no overflow.
- `Fs == Fc` — Stribeck term vanishes, pure Coulomb+viscous.
- Negative-going zero crossing — output stays continuous (no chatter).
