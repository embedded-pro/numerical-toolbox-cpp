# Gain-Scheduled Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestGainScheduledController : public ::testing::Test:
    # 3 breakpoints at 0.0, 0.5, 1.0; single gain per point
    using Point = SchedulePoint<float, 1>
    std::array<Point,3> table{{ {0.0f,{1.0f}},
                                {0.5f,{2.0f}},
                                {1.0f,{4.0f}} }}
    GainScheduledController<float,3,1> scheduler{ table }
# each case below is a TEST_F(TestGainScheduledController, <name>)
```

## Test cases (Arrange / Act / Assert)

```
returns_exact_gain_at_breakpoint:
    Arrange: s = 0.5
    Assert:  active gain == 2.0

interpolates_midway_between_points:
    Arrange: s = 0.25 (halfway in first interval)
    Assert:  active gain ≈ 1.5

interpolates_in_second_interval:
    Arrange: s = 0.75
    Assert:  active gain ≈ 3.0

saturates_below_first_breakpoint:
    Arrange: s = -1.0
    Assert:  active gain == 1.0 (first point held)

saturates_above_last_breakpoint:
    Arrange: s = 2.0
    Assert:  active gain == 4.0 (last point held)

selects_correct_interval:
    Arrange: s just above 0.5
    Assert:  blend uses points [0.5, 1.0], not [0.0, 0.5]

multi_gain_vectors_blend_componentwise:
    Arrange: GainSize = 2, each component interpolated independently
    Assert:  both components match their own linear blend

active_gains_persist_between_calls:
    Arrange: Schedule(0.25) then read ActiveGains()
    Assert:  returns last interpolated set (≈1.5)
```

## Reference vectors

- Table {(0,1),(0.5,2),(1,4)}: `s=0.25 → 1.5`, `s=0.75 → 3.0`, `s=0.5 → 2.0` exactly.
- Outside `[0,1]` the gain is held at the nearest endpoint (no extrapolation).

## Edge cases

- Scheduling variable exactly on a breakpoint returns the stored gain (no interpolation error).
- Two-point table degenerates to a single linear segment.
