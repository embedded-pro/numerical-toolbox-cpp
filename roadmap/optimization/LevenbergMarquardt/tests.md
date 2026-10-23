# Levenberg–Marquardt — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestLevenbergMarquardt : public ::testing::Test:
    RosenbrockResiduals rosenbrock      # r = (10(y − x²), 1 − x), J = [[−20x, 10], [−1, 0]], Retract = add
    UnitCircleAngle     circle{ target } # State = angle θ; r = (cos θ, sin θ) − target; J = (−sin θ, cos θ)ᵀ;
                                         # Retract = θ + δ wrapped to (−π, π]
    ConstantDamping<float, 2, 2> gaussNewton{ 0 }
    StrictMock<DampingPolicyMock<float, 2, 2>> policy
    LevenbergMarquardtConfig<float> config{ .tolerance = 1e-5, .maxStep = +∞, .maxIterations = 20 }
# each case below is a TEST_F(TestLevenbergMarquardt, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
gauss_newton_solves_rosenbrock_residuals_in_two_steps:
    Act:    LevenbergMarquardt{ rosenbrock, gaussNewton, config }.Solve((−1.2, 1))
    Assert: x ≈ (1, 1), converged, iterations == 2
damping_policy_is_queried_with_the_current_jacobian:
    Arrange: config.maxIterations = 1;  EXPECT_CALL(policy, DampingSquared(J(−1.2, 1) = [[24, 10], [−1, 0]])) → 0
    Assert:  x ≈ (1, −3.84), !converged
step_is_clamped_to_max_step:
    Arrange: config.maxIterations = 1, config.maxStep = 1
    Assert:  x ≈ (−0.786197, 0.089634)  (undamped step (2.2, −4.84) scaled to unit length)
retraction_keeps_a_wrapped_state:
    Arrange: circle target (cos 2.5, sin 2.5), θ₀ = −2.5 (1.28 rad away across ±π)
    Assert:  θ ≈ 2.5, converged, θ ∈ (−π, π] after every step (first step lands at 2.824261 via the wrap)
unreachable_target_reports_non_convergence:
    Arrange: circle target (2, 0), ConstantDamping<float, 2, 1>{ 1 }, θ₀ = 0.3
    Assert:  !converged, iterations == maxIterations, θ ≈ 0, residualNorm ≈ 1
singular_undamped_system_stops_without_dividing_by_zero:
    Arrange: constant residual (1, 1), J ≡ 0, ConstantDamping{ 0 }
    Assert:  !converged, iterations == 0, x == x₀
```

## Reference vectors

- Rosenbrock from `(−1.2, 1)`, Gauss–Newton: `(1, −3.84)` after one step, `(1, 1)` after two (exact).
- First undamped step `(2.2, −4.84)`, norm `5.316540`.
- Circle, `θ₀ = −2.5`, target angle `2.5`: `θ = 2.824261 → 2.505653 → 2.500000` (python3).
- Unreachable target, `λ² = 1`: `θ ← θ − sin θ` ⇒ `θ → 0`, residual `‖(1, 0) − (2, 0)‖ = 1`.

## Edge cases

- `maxIterations = 0` ⇒ returns `x₀` with its residual, `converged` iff already within tolerance.
- Tall problem (`Residuals > Parameters`) and wide/square problem exercise the two solve branches.
