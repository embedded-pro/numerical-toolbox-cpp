# H∞ State-Feedback Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestHInfinityStateFeedback : public ::testing::Test:
    # 2-state plant, 1 disturbance input, 1 control input, 1 error output
    GeneralizedPlant<float,2,1,1,1> plant = MakeGeneralizedPlant()
    HInfinityStateFeedback<float,2,1,1,1> hinf{ plant }
# each case below is a TEST_F(TestHInfinityStateFeedback, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reduces_to_lqr_as_gamma_large:
    Arrange: Synthesize with a very large γ range (weak antagonist)
    Assert:  K ≈ the LQR gain on (A, B2, C1ᵀC1, I)

closed_loop_is_schur_stable:
    Arrange: successful Synthesize
    Assert:  every eigenvalue of (A − B2·K) lies inside the unit disk

achieves_target_gamma:
    Arrange: synthesize at γ, simulate worst-case disturbance
    Assert:  ‖z‖ / ‖w‖ <= γ  (attenuation met)

bisection_finds_minimal_gamma:
    Arrange: scalar/2-state plant with known optimal γ*
    Assert:  returned gamma ≈ γ* within tol

infeasible_below_gamma_optimum:
    Arrange: request γ < γ*
    Assert:  RiccatiFeasible(γ) == false; Synthesize reports failure

gain_matches_gare_solution:
    Arrange: solve the game Riccati directly for a known plant
    Assert:  K equals the control block of (R̃ + BᵀXB)^{-1} BᵀXA

rejects_worst_case_disturbance:
    Arrange: drive the loop with the maximizing disturbance
    Assert:  output energy bounded by γ²·‖w‖²

compute_control_is_negative_feedback:
    Arrange: nonzero state x
    Assert:  u == -K·x  (correct sign)
```

## Reference vectors

- Scalar plant: closed-form `γ*` and gain — the golden values for `bisection_finds_minimal_gamma`.
- `γ → ∞`: the H∞ gain converges to the LQR gain (golden LQR comparison).

## Edge cases

- `γ` below `γ*` — no PSD stabilizing solution; assert clean infeasibility, not divergence.
- Disturbance block `(−γ²I + B1ᵀX B1)` losing definiteness — assert `RiccatiFeasible` rejects `g`.
- Near `γ*` the Riccati is ill-conditioned — assert the bisection tolerance guards against it.
