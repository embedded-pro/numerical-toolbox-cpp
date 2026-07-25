# Disturbance Observer — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestDisturbanceObserver : public ::testing::Test:
    # first-order nominal plant Pn + first-order low-pass Q with unity DC gain
    math::LinearTimeInvariant<float,1,1,1> nominal = MakeFirstOrderPlant()
    QFilterCoeffs q = MakeLowPassQ(cutoffHz = 20.0f, sampleRateHz = 1000.0f)
    DisturbanceObserver<float,1,1,1> dob{ nominal, q }
# each case below is a TEST_F(TestDisturbanceObserver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
estimates_constant_disturbance:
    Arrange: inject a constant input disturbance d, run to steady state
    Assert:  Disturbance() -> d  (DC estimate converges)

cancels_disturbance_at_output:
    Arrange: closed loop with a constant load disturbance
    Assert:  output returns to the nominal (disturbance-free) trajectory

transparent_outside_q_bandwidth:
    Arrange: high-frequency disturbance above Q's cutoff
    Assert:  d̂ ≈ 0, control ≈ nominal c (DOB does not act)

unity_dc_gain_q_filter:
    Arrange: constant input to Q
    Assert:  Q output -> input  (Q(1) == 1, full DC rejection)

recovers_nominal_plant_behaviour:
    Arrange: actual plant = Pn with a gain perturbation, DOB active
    Assert:  input–output response matches nominal Pn within tolerance

proper_realization_no_differentiation:
    Arrange: step of measurement noise into Q·Pn^{-1}
    Assert:  response is bounded (no derivative spike) — properness holds

reset_clears_estimate:
    Arrange: accumulate a disturbance estimate, Reset()
    Assert:  Disturbance() == 0 and internal states cleared

robust_to_small_model_mismatch:
    Arrange: Pn off by a few percent from the true plant
    Assert:  steady-state disturbance effect still reduced (not amplified)
```

## Reference vectors

- DC: `d̂ = Q(1)·Pn^{-1}(1)·y − Q(1)·u = d` for a constant disturbance (golden equality).
- Q-filter step response equals the standalone `BiquadCascade` golden output.

## Edge cases

- Q bandwidth too wide — noise amplification; assert the documented robustness/noise trade-off.
- Non-minimum-phase `Pn` — `Pn^{-1}` unstable; assert the stable-inverse handling / precondition.
- Large model mismatch — closed loop can destabilize; document the robust-stability limit.
