# S-Curve (Jerk-Limited) Profile — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSCurveProfile : public ::testing::Test:
    MotionLimits<float> limits{ .vMax = 1.0f, .aMax = 2.0f, .jMax = 10.0f }
    SCurveProfile<float> profile{ 0.0f, 5.0f, limits }   # full 7-segment move
# each case below is a TEST_F(TestSCurveProfile, <name>)
```

## Test cases (Arrange / Act / Assert)

```
endpoints_reached_exactly:
    Assert: Sample(0).position ≈ 0  and  Sample(tf).position ≈ 5

endpoints_at_rest_and_zero_accel:
    Assert: Sample(0){vel,acc} ≈ 0  and  Sample(tf){vel,acc} ≈ 0

velocity_never_exceeds_vmax:
    Assert: max |velocity| ≤ vMax (1.0) across the profile

acceleration_never_exceeds_amax:
    Assert: max |acceleration| ≤ aMax (2.0) across the profile

jerk_never_exceeds_jmax:
    Assert: every sampled |jerk| ∈ {0, jMax}, ≤ jMax (10.0)

acceleration_is_continuous:
    Arrange: dense sampling across phase joins
    Assert:  no acceleration step between adjacent samples (bounded by jMax·dt)

short_move_skips_cruise:
    Arrange: q0=0, qf=0.05 (very short)
    Assert:  ReachesMaxVel() == false, endpoints still reached

sample_clamps_outside_domain:
    Assert: Sample(-1) == Sample(0)  and  Sample(tf+1) == Sample(tf)
```

## Reference vectors

- Full profile `vMax=1, aMax=2, jMax=10`: jerk phase `Tj = aMax/jMax = 0.2 s`;
  accel phase `Ta = Tj + vMax/aMax = 0.7 s`.
- Rest-to-rest S-curve is symmetric ⇒ velocity peak occurs at `tf/2`.

## Edge cases

- `jMax → ∞` ⇒ jerk phases vanish, profile degenerates to a trapezoid (cross-check limit).
- `vMax`/`aMax` unreachable for a short move ⇒ 4- or 2-segment degenerate profile, still valid.
- `d == 0` ⇒ zero-length profile; `Sample(0)` is the start, fully at rest.
