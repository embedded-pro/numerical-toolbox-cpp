# Bang-Bang / Hysteresis (Relay) Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestBangBangHysteresis : public ::testing::Test:
    # band [-0.2, 0.2], outputs {0, 1}
    BangBangHysteresis<float> relay{ -0.2f, 0.2f, 0.0f, 1.0f }
# each case below is a TEST_F(TestBangBangHysteresis, <name>)
```

## Test cases (Arrange / Act / Assert)

```
starts_in_low_state:
    Assert: State() == Low and Update(0) == outputLow

switches_high_at_upper_threshold:
    Arrange: from Low, feed x = 0.2
    Assert:  output becomes outputHigh, State() == High

stays_high_inside_band:
    Arrange: from High, feed x = 0.0 (inside band)
    Assert:  output remains outputHigh (no premature switch)

switches_low_at_lower_threshold:
    Arrange: from High, feed x = -0.2
    Assert:  output becomes outputLow, State() == Low

hysteresis_prevents_chatter:
    Arrange: feed a sequence dithering around 0 within (-0.2, 0.2)
    Assert:  no state change occurs

full_cycle_sequence:
    Arrange: x = [0, 0.3, 0.1, -0.3, 0.0]
    Assert:  states = [Low, High, High, Low, Low]

reset_restores_initial_state:
    Arrange: drive to High, Reset(Low)
    Assert:  State() == Low

custom_output_levels:
    Arrange: outputs {-1, +1}
    Assert:  Low emits -1, High emits +1
```

## Reference vectors

- band [-0.2, 0.2]: a rising input crosses to High only at `x ≥ 0.2`; a falling input returns to
  Low only at `x ≤ -0.2`.
- Inside `(-0.2, 0.2)` the output equals its previous value (memory / latch behaviour).

## Edge cases

- Exactly-on-threshold values (`x == highThreshold`, `x == lowThreshold`) switch (inclusive).
- Zero-width band collapses to a comparator (still valid but no hysteresis).
- Custom levels including negative outputs must pass through unchanged.
