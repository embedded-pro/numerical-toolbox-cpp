# Trapezoidal (LSPB) Velocity Profile — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestTrapezoidalProfile : public ::testing::Test:
    MotionLimits<float> limits{ .vMax = 1.0f, .aMax = 2.0f }
    TrapezoidalProfile<float> profile{ 0.0f, 5.0f, limits }   # long move ⇒ trapezoid
# each case below is a TEST_F(TestTrapezoidalProfile, <name>)
```

## Test cases (Arrange / Act / Assert)

```
endpoints_reached_exactly:
    Assert: Sample(0).position ≈ 0  and  Sample(tf).position ≈ 5

endpoints_are_at_rest:
    Assert: Sample(0).velocity ≈ 0  and  Sample(tf).velocity ≈ 0

cruise_velocity_is_clamped_to_vmax:
    Arrange: long move
    Assert:  max velocity over the profile ≈ vMax (1.0), never exceeds it

acceleration_is_within_limit:
    Assert: |acceleration| ≤ aMax at all sampled instants

short_move_is_triangular:
    Arrange: q0=0, qf=0.1 (d < vMax²/aMax)
    Assert:  IsTriangular() == true  and  peak velocity < vMax

phase_durations_match_trapezoid_formula:
    Arrange: d=5, vMax=1, aMax=2
    Assert:  tAccel ≈ 0.5, tCruise ≈ 4.5, Duration ≈ 5.5

negative_direction_move:
    Arrange: q0=2, qf=0
    Assert:  velocity ≤ 0 throughout, endpoints (2 → 0) reached

sample_clamps_outside_domain:
    Assert: Sample(-1) == Sample(0)  and  Sample(tf+1) == Sample(tf)
```

## Reference vectors

- `d=5, vMax=1, aMax=2`: `tAccel = vMax/aMax = 0.5`, `tCruise = (d − vMax²/aMax)/vMax = 4.5`,
  `tf = 5.5`.
- Triangular threshold distance `= vMax²/aMax = 0.5`.

## Edge cases

- `d` exactly equal to `vMax²/aMax` ⇒ zero cruise time (trapezoid degenerates to triangle).
- `d == 0` ⇒ `tf == 0`, `Sample(0)` is the start at rest.
- `aMax` very large ⇒ ramps vanish, profile approaches a pure velocity step.
