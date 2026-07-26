# Median Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestMedianFilter : public ::testing::Test:
    MedianFilter<float, 3> med{}           # window length N = 3
# each case below is a TEST_F(TestMedianFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
rejects_single_impulse:
    Arrange: N = 3, input = [0.1, 0.1, 0.9, 0.1, 0.1]  (spike at index 2)
    Act:     collect y
    Assert:  spike suppressed — median output stays ≈ 0.1

preserves_step_edge:
    Arrange: input = [0, 0, 1, 1, 1]
    Assert:  output steps 0 -> 1 without overshoot or smearing

constant_input_passthrough:
    Arrange: feed constant c
    Assert:  output == c for all samples after warm-up

sorted_middle_is_returned:
    Arrange: window = [0.3, 0.1, 0.2]
    Assert:  output == 0.2  (true median, not mean)

majority_spike_passes:
    Arrange: N = 3, input = [0, 0.9, 0.9, 0]  (spike is a majority)
    Assert:  median follows the majority (0.9) — documents the N/2 rejection limit

reset_clears_window:
    Arrange: run samples, Reset(0)
    Assert:  window re-seeded to 0; median == 0 until new data dominates
```

## Reference vectors

- `N = 3`, `[2, 80, 6]` (unsorted) ⇒ median `6`.
- `N = 5`, `[5, 5, 100, 5, 5]` ⇒ median `5` (impulse fully rejected).

## Edge cases

- `N = 1` ⇒ pass-through.
- Monotonic ramp input: output tracks with a fixed `(N−1)/2`-sample lag.
