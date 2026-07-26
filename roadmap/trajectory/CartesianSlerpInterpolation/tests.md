# Cartesian Path + Orientation (SLERP) Interpolation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCartesianSlerp : public ::testing::Test:
    Pose<float> start{ .position = {0,0,0}, .orientation = Identity }
    Pose<float> goal { .position = {1,0,0}, .orientation = rot(ẑ, 90°) }
    CartesianSlerpInterpolation<float> path{ start, goal, 1.0f }
# each case below is a TEST_F(TestCartesianSlerp, <name>)
```

## Test cases (Arrange / Act / Assert)

```
endpoints_match_start_and_goal:
    Assert: Sample(0) ≈ start  and  Sample(tf) ≈ goal

position_is_straight_line:
    Arrange: goal.position = (1,0,0)
    Assert:  Sample(tf/2).position ≈ (0.5, 0, 0)

orientation_is_unit_at_midpoint:
    Assert: |Sample(tf/2).orientation| ≈ 1  (SLERP preserves unit norm)

slerp_midpoint_is_half_angle:
    Arrange: 90° rotation about ẑ
    Assert:  Sample(tf/2).orientation ≈ rot(ẑ, 45°)

shortest_path_negates_goal:
    Arrange: goal.q with negative dot to start.q (obtuse)
    Assert:  interpolation takes the < 180° arc (angle decreases monotonically)

near_parallel_uses_nlerp:
    Arrange: goal.q within 0.01° of start.q
    Assert:  no NaN/inf (no 1/sinθ blow-up), result ≈ start.q

constant_angular_rate_for_linear_scaling:
    Arrange: linear s(t)
    Assert:  angle(start, Sample(t)) grows linearly in t

sample_clamps_outside_domain:
    Assert: Sample(-1) ≈ start  and  Sample(tf+1) ≈ goal
```

## Reference vectors

- SLERP of Identity → `rot(ẑ, 90°)` at `s = 0.5` ⇒ `rot(ẑ, 45°)`.
- Straight-line position `p(s) = p0 + s(p1 − p0)`; midpoint of `(0,0,0)→(1,0,0)` is `(0.5,0,0)`.

## Edge cases

- Identical start and goal orientation ⇒ constant quaternion, `nlerp` path, no division by zero.
- Antipodal quaternions (`dot ≈ −1`) ⇒ shortest-path negation, well-defined 180° arc.
- Zero-length Cartesian move with pure rotation ⇒ position constant, orientation still slerps.
