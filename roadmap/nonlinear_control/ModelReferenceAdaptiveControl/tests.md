# Model Reference Adaptive Control (MRAC) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestModelReferenceAdaptiveControl : public ::testing::Test:
    # First-order reference model  ẋ_m = -a_m x_m + b_m r  (stable, a_m > 0):
    math::LinearTimeInvariant<float,1,1,1> refModel = MakeFirstOrderReference()
    ModelReferenceAdaptiveControl<float,1,1> mrac{ refModel, /*gamma*/1.0,
                                                   /*signB*/+1.0, AdaptationLaw::Lyapunov }
# each case below is a TEST_F(TestModelReferenceAdaptiveControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reference_model_advances:
    Arrange: xm=0, step command r=1
    Act:     ComputeControl(x, r, dt) once
    Assert:  xm == (refModel.B · r) · dt      (Euler step of the reference model)

zero_error_freezes_parameters:
    Arrange: x == xm so e = 0
    Assert:  thetaX and thetaR unchanged after the update

positive_error_adapts_feedback:
    Arrange: e = x - xm > 0, x > 0, signB = +1
    Assert:  thetaX decreases by gamma·e·x·dt

feedforward_param_tracks_command:
    Arrange: e != 0, r != 0
    Assert:  thetaR changes by -gamma·signB·e·r·dt

signB_flips_adaptation_direction:
    Arrange: identical error/state, signB = -1
    Assert:  parameter update has opposite sign vs signB = +1

control_law_combines_terms:
    Arrange: known thetaX, thetaR, x, r
    Assert:  u == thetaX·x + thetaR·r

tracks_reference_over_time:
    Arrange: wrap an unknown first-order plant; run K steps with a step command
    Assert:  |x - xm| -> 0 (below tol); Lyapunov law keeps signals bounded

gamma_scales_adaptation_speed:
    Arrange: two runs, gamma = 0.5 vs 5.0
    Assert:  larger gamma reduces tracking error faster (until it destabilizes — documented)

parameters_converge_under_excitation:
    Arrange: persistently-exciting r (sum of sinusoids), known plant params
    Assert:  thetaX, thetaR approach the ideal matching gains

reset_clears_state_and_params:
    Arrange: adapt, then Reset()
    Assert:  xm == 0, thetaX == 0, thetaR == 0
```

## Reference vectors

- Ideal matching gains: for plant `ẋ = a·x + b·u` and reference `ẋ_m = -a_m·x_m + b_m·r`,
  the perfect params are `θ_x* = -(a_m + a)/b`, `θ_r* = b_m/b` — the convergence targets.
- Zero error ⇒ `θ̇ = 0`; the golden invariant for the freeze test.

## Edge cases

- Large `gamma` with MIT rule: expect (and assert) growth — demonstrates the stability caveat.
- Disturbance with no σ-modification: parameters drift; enabling modification bounds them.
- No excitation (constant `r`): tracking succeeds but parameters need not reach `θ*`.
