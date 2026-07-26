# Saturation / Rate-Limiter / Slew Blocks — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSlewLimitedSaturation : public ::testing::Test:
    Saturation<float>  sat{ -0.5f, 0.5f }
    RateLimiter<float> slew{ 0.1f, 1.0f }    # 0.1 /s, Ts = 1 s -> step 0.1
# each case below is a TEST_F(TestSlewLimitedSaturation, <name>)
```

## Test cases (Arrange / Act / Assert)

```
clamp_passes_value_inside_bounds:
    Arrange: bounds [-0.5, 0.5], u = 0.25
    Assert:  Clamp(u) == 0.25

clamp_saturates_above_and_below:
    Arrange: bounds [-0.5, 0.5]
    Assert:  Clamp(0.9) == 0.5 and Clamp(-0.9) == -0.5

rate_limiter_first_sample_seeds_state:
    Arrange: fresh limiter, step 0.1
    Act:     first call Limit(0.8)
    Assert:  returns 0.8 (no limiting on prime)

rate_limiter_caps_positive_slope:
    Arrange: prime at 0, then request 1.0 each tick
    Assert:  outputs rise by 0.1 per tick: 0.1, 0.2, 0.3, ...

rate_limiter_caps_negative_slope:
    Arrange: prime at 0, then request -1.0 each tick
    Assert:  outputs fall by 0.1 per tick: -0.1, -0.2, ...

rate_limiter_follows_small_changes_exactly:
    Arrange: step 0.1, request a delta of 0.05
    Assert:  output tracks input with no lag

reset_reprimes_state:
    Arrange: run some samples, Reset(0.2)
    Assert:  next small change starts from 0.2

composition_slew_then_clamp_honours_both:
    Arrange: bounds [-0.5,0.5], step 0.1, step-request to 10
    Assert:  output ramps by 0.1 and never exceeds 0.5
```

## Reference vectors

- step 0.1, request `+∞`, prime 0 ⇒ output `= 0.1·n` until saturation.
- bounds [-0.5, 0.5] ⇒ `Clamp` is the identity on `[-0.5, 0.5]`.

## Edge cases

- `maxRate = 0` freezes the output at the primed value.
- `lo == hi` collapses the output to a constant.
- Large step requests ramp by `rate·Ts` per tick with no overshoot (`EXPECT_NEAR`, tol `1e-6f`).
