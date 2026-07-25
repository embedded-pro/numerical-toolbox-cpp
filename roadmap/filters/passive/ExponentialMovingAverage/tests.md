# Exponential Moving Average — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestExponentialMovingAverage : public ::testing::Test:
    ExponentialMovingAverage<float> ema{ 0.5f }   # alpha = 0.5 default
# each case below is a TEST_F(TestExponentialMovingAverage, <name>)
```

## Test cases (Arrange / Act / Assert)

```
impulse_decays_geometrically:
    Arrange: alpha = 0.5, input = [1, 0, 0, 0]
    Act:     collect y for each sample
    Assert:  y ≈ [0.5, 0.25, 0.125, 0.0625]  (EXPECT_NEAR, tol 1e-6f)

constant_input_converges_to_dc:
    Arrange: alpha = 0.25, feed constant c = 0.8 for 50 samples
    Assert:  final output ≈ 0.8  (unity DC gain)

alpha_one_is_passthrough:
    Arrange: alpha = 1.0
    Assert:  Filter(x) == x for a sequence

step_response_matches_time_constant:
    Arrange: alpha = 0.1, step from 0 -> 1
    Assert:  after ~1 time-constant, output ≈ 0.63 (±tol)

reset_clears_state:
    Arrange: run some samples, Reset(0)
    Assert:  next output == alpha * next_input

disabled_passes_through:
    Arrange: Disable()
    Assert:  Filter(x) == x, state unchanged
```

## Reference vectors

- `alpha = 0.5`, unit impulse ⇒ `y[n] = 0.5^(n+1)`.
- `alpha = 0.5`, unit step ⇒ `y[n] = 1 - 0.5^(n+1)`.

## Edge cases

- `alpha` at bounds (→0 very slow, =1 passthrough).
- Near-zero and unity `alpha` remain numerically stable.
- Long constant-input run converges to the DC value without drift.
