# Integral State Feedback (LQI) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestIntegralStateFeedbackLqi : public ::testing::Test:
    # 2-state SISO plant, single tracked output, Ts = 0.01
    math::LinearTimeInvariant<float,2,1,1> plant = MakeFirstOrderPlant()
    math::SquareMatrix<float,3> Q = ...      # augmented: 2 states + 1 integral
    math::SquareMatrix<float,1> R = ...
    IntegralStateFeedbackLqi<float,2,1,1> controller{ plant, Q, R, 0.01f }
# each case below is a TEST_F(TestIntegralStateFeedbackLqi, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_steady_state_error_to_step_reference:
    Arrange: constant reference r, closed-loop simulate to steady state
    Assert:  measured output -> r (error -> 0)

rejects_constant_output_disturbance:
    Arrange: add a constant disturbance to the output
    Assert:  integral action drives steady-state error back to 0

integral_accumulates_error:
    Arrange: constant non-zero error for K ticks
    Assert:  integral == sum(error·Ts) over those ticks

gain_split_matches_augmented_lqr:
    Arrange: known plant + weights
    Assert:  [Kx | Ki] equals the Lqr gain on the augmented (Aa,Ba)

closed_loop_is_stable:
    Arrange: simulate x[k+1] = (A - B·Kx)x - B·Ki·∫e
    Assert:  states remain bounded and converge

reset_clears_integral:
    Arrange: accumulate integral, Reset()
    Assert:  integral == 0, next control uses the state term only

reference_change_tracked:
    Arrange: step reference from r1 to r2 mid-run
    Assert:  output converges to r2 with zero final error

control_uses_negative_feedback:
    Arrange: positive state and positive integral
    Assert:  u = -(Kx·x + Ki·∫e) (correct sign)
```

## Reference vectors

- At equilibrium `r − y = 0` ⇒ integral constant ⇒ unique `u` holding `y = r`.
- The augmented-plant LQR gain `[Kx|Ki]` computed offline is the golden vector.

## Edge cases

- Actuator saturation without anti-windup — document integral windup; verify the clamped variant.
- Marginally stable plant (integrator in the plant) — augmentation still yields finite gains.
- Reference outside the reachable range — bounded steady-state error, no divergence.
