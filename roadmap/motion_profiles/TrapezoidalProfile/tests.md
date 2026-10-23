# Trapezoidal (LSPB) Velocity Profile — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestTrapezoidalProfile : public ::testing::Test:
    MotionLimits<float> limits{ .vMax = 1.0f, .aMax = 2.0f }
    TrapezoidalProfile<float> profile{ 0.0f, 5.0f, limits }   # long move ⇒ trapezoid
# each case below is a TEST_F(TestTrapezoidalProfile, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
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

plan_with_duration_lowers_cruise_speed:
    Arrange: PlanWithDuration(0, 4, aMax=1, duration=5)
    Assert:  has_value; Duration() ≈ 5; Sample(0.5).velocity ≈ 0.5 (ramp at aMax);
             Sample(2.5).velocity ≈ 1 (cruise vPeak = 1 < aMax·T/2); Sample(5).position ≈ 4

plan_with_duration_rejects_infeasible_time:
    Arrange: PlanWithDuration(0, 4, aMax=1, duration=3)      # aMax·T² = 9 < 4d = 16
    Assert:  == nullopt

sample_clamps_outside_domain:
    Assert: Sample(-1) == Sample(0)  and  Sample(tf+1) == Sample(tf)
```

## Reference vectors

- `d=5, vMax=1, aMax=2`: `tAccel = vMax/aMax = 0.5`, `tCruise = (d − vMax²/aMax)/vMax = 4.5`,
  `tf = 5.5`.
- Triangular threshold distance `= vMax²/aMax = 0.5`.
- `PlanWithDuration`, `d=4, aMax=1, T=5`: `v = (aMax·T − sqrt(aMax²T² − 4·aMax·d))/2 = (5 − 3)/2 = 1`,
  `tAccel = 1`, `tCruise = 3`. `d=5, aMax=2, T=6`: `v = 0.900980`, `tAccel = 0.450490`,
  `tCruise = 5.099020`. Feasible iff `aMax·T² ≥ 4d` (equality ⇒ triangle, `v = aMax·T/2`).
- Synchronization: replanning with `(vMax/λ, aMax/λ²)` scales `Duration()` by exactly `λ`
  (e.g. fixture, `λ = 2` ⇒ `11.0`).

## Edge cases

- `d` exactly equal to `vMax²/aMax` ⇒ zero cruise time (trapezoid degenerates to triangle).
- `d == 0` ⇒ `tf == 0`, `Sample(0)` is the start at rest.
- `aMax` very large ⇒ ramps vanish, profile approaches a pure velocity step.
- `PlanWithDuration` with `d == 0` ⇒ `vPeak = 0`, holds `q0` at rest for the whole duration.
