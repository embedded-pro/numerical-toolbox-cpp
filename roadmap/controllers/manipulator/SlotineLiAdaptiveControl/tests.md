# Slotine-Li Adaptive Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSlotineLiAdaptiveControl : public ::testing::Test:
    # regressor injected & mocked; Dof = 2, NumParams = 3:
    StrictMock<dynamics::MockInertialRegressor<float,2,3>> regressor
    SquareMatrix<float,2> Lambda = diag(5, 5)
    SquareMatrix<float,2> Kd     = diag(20, 20)
    SquareMatrix<float,3> Gamma  = diag(1, 1, 1)
    Vector<float,3>       aHat0  = {0, 0, 0}
    float dt = 0.001
    SlotineLiAdaptiveControl<float,2,3> controller{ regressor, Lambda, Kd, Gamma, aHat0, dt }
# each case below is a TEST_F(TestSlotineLiAdaptiveControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_error_uses_feedforward_only:
    Arrange: q=qd, qDot=qdDot ⇒ s=0; regressor Y given
    Act:     τ = ComputeTorque(...)
    Assert:  τ == Y · aHat   (no −Kd·s term)

sliding_variable_damping:
    Arrange: s != 0 via qTildeDot, regressor Y = 0
    Assert:  τ == -Kd · s

sliding_variable_definition:
    Arrange: known qTilde, qTildeDot
    Assert:  s == qTildeDot + Lambda · qTilde

reference_motion_uses_lambda:
    Arrange: known errors
    Assert:  regressor called with qrDot == qdDot − Λ·qTilde and qrDdot == qdDdot − Λ·qTildeDot

parameter_estimate_adapts:
    Arrange: s != 0, Y != 0, one step
    Assert:  aHat == aHat0 − Γ·Yᵀ·s·dt

no_adaptation_on_zero_sliding:
    Arrange: s = 0
    Assert:  aHat unchanged after the step

estimate_persists_across_calls:
    Arrange: two steps with the same s, Y
    Assert:  aHat integrates twice; ParameterEstimate() reflects both

convergence_tracks_trajectory:
    Arrange: wrap a plant M q̈ + Cq̇ + g = τ with unknown a; run K steps
    Assert:  ||q − qd|| -> 0 (tracking) even though aHat != a

reset_restores_initial_estimate:
    Arrange: adapt, then Reset(aHat0)
    Assert:  ParameterEstimate() == aHat0

regressor_queried_once_per_call:
    Arrange: any state
    Assert:  regressor.Compute called exactly once (StrictMock)
```

## Reference vectors

- By construction `s = q̃̇ + Λq̃`; single joint, `Λ=λ`, constant `q̃=e`, `q̃̇=0` ⇒ `s = λe` (hand-checkable).
- With `aHat = a` (true params) the law reduces to computed-torque-like `τ = Y·a − Kd·s`.
- One adaptation step: `Δa = −Γ·Yᵀ·s·dt` — exact, hand-verifiable.

## Edge cases

- Non-persistent excitation: `aHat` settles on a manifold; tracking still converges.
- Large `Γ`: estimate oscillation — assert bounded, no divergence.
- Parameter drift under noisy `s`: projection / dead-zone keeps `aHat` physical.
