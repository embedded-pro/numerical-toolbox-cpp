# Moving Average — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestMovingAverage : public ::testing::Test:
    MovingAverage<float, 4> ma{}           # window length N = 4
# each case below is a TEST_F(TestMovingAverage, <name>)
```

## Test cases (Arrange / Act / Assert)

```
warmup_fills_window:
    Arrange: N = 4, input = [1, 0, 0, 0], zero-initialized window
    Act:     collect y per sample
    Assert:  y ≈ [0.25, 0.25, 0.25, 0.25]  (impulse spread over N taps)

constant_input_unity_gain:
    Arrange: feed constant c = 0.5 for > N samples
    Assert:  steady-state output ≈ 0.5

step_response_is_linear_ramp:
    Arrange: step 0 -> 1 with N = 4
    Assert:  y climbs 0.25, 0.5, 0.75, 1.0 then holds

running_sum_matches_direct_average:
    Arrange: random bounded sequence
    Act:     compare Filter() against a brute-force mean of the last N
    Assert:  equal within tolerance  (EXPECT_NEAR, tol 1e-6f)

impulse_leaves_after_N:
    Arrange: single impulse then zeros
    Assert:  output returns to 0 exactly after N samples (FIR, finite memory)

reset_clears_state:
    Arrange: run samples, Reset(0)
    Assert:  window and sum cleared; next output == input / N
```

## Reference vectors

- `N = 4`, unit impulse ⇒ `y = [0.25, 0.25, 0.25, 0.25, 0, 0, ...]`.
- `N = 4`, unit step ⇒ `y = [0.25, 0.5, 0.75, 1, 1, ...]`.

## Edge cases

- `N = 1` ⇒ pass-through.
- Long constant run: assert no slow drift from running-sum rounding (`EXPECT_NEAR`, tol `1e-6f`).
