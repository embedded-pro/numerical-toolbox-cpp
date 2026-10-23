# S-Curve (Jerk-Limited) Profile — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSCurveProfile : public ::testing::Test:
    MotionLimits<float> limits{ .vMax = 1.0f, .aMax = 2.0f, .jMax = 10.0f }
    SCurveProfile<float> profile{ 0.0f, 5.0f, limits }   # full 7-segment move (step 1, aMax reached)
# each case below is a TEST_F(TestSCurveProfile, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
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

step1_full_profile_durations:                 # vMax·jMax ≥ aMax², vMax and aMax reached
    Assert:  Duration() ≈ 5.7; Sample(0.2).acceleration ≈ 2 (end of jerk phase, Tj = 0.2);
             Sample(0.7).velocity ≈ 1 (end of accel phase, Ta = 0.7);
             ReachesMaxAccel() and ReachesMaxVel()

step1_triangular_acceleration_when_jerk_low:  # vMax·jMax < aMax²
    Arrange: limits { vMax=1, aMax=2, jMax=1 }, q0=0, qf=5
    Assert:  Duration() ≈ 7; Sample(1).acceleration ≈ 1 (peak jMax·Tj < aMax);
             Sample(3.5).velocity ≈ 1; !ReachesMaxAccel() and ReachesMaxVel()

step2_short_move_reaches_amax_not_vmax:
    Arrange: q0=0, qf=0.3 (fixture limits)
    Assert:  Duration() ≈ 1.0; Sample(0.5).velocity ≈ 0.6 (vLim, at tf/2);
             max |acceleration| ≈ 2; ReachesMaxAccel() and !ReachesMaxVel(); Sample(tf).position ≈ 0.3

step3_very_short_move_reaches_neither:
    Arrange: q0=0, qf=0.02 (fixture limits)
    Assert:  Duration() ≈ 0.4; Sample(0.1).acceleration ≈ 1 (aLim = jMax·Tj);
             Sample(0.2).velocity ≈ 0.1 (vLim); !ReachesMaxAccel() and !ReachesMaxVel();
             Sample(tf).position ≈ 0.02

negative_direction_mirrors_profile:
    Arrange: q0=5, qf=0
    Assert:  Sample(t) = { 5 − p(t), −v(t), −a(t), −j(t) } of the fixture profile

sample_clamps_outside_domain:
    Assert: Sample(-1) == Sample(0)  and  Sample(tf+1) == Sample(tf)
```

## Reference vectors

Computed by integrating the 7-segment constant-jerk profile (python3, exact): each ends at `h` with
zero velocity/acceleration and respects `vMax`, `aMax`, `jMax`.

| Case       | Limits `(vMax, aMax, jMax)` | `h`  | Branch        | `Tj` | `Ta` | `Tv` | `tf` | `aLim` | `vLim` |
|------------|-----------------------------|------|---------------|------|------|------|------|--------|--------|
| full       | (1, 2, 10)                  | 5    | 1 (`vj ≥ a²`) | 0.2  | 0.7  | 4.3  | 5.7  | 2      | 1      |
| low jerk   | (1, 2, 1)                   | 5    | 1 (`vj < a²`) | 1.0  | 2.0  | 3.0  | 7.0  | 1      | 1      |
| short      | (1, 2, 10)                  | 0.3  | 2             | 0.2  | 0.5  | 0    | 1.0  | 2      | 0.6    |
| very short | (1, 2, 10)                  | 0.02 | 3             | 0.1  | 0.2  | 0    | 0.4  | 1      | 0.1    |

- Step 2 for `h = 0.3`: `Δ = 2⁴/10² + 4·2·0.3 = 2.56`, `Ta = (0.4 + 1.6)/4 = 0.5 ≥ 2·Tj = 0.4`.
- Step 3 for `h = 0.02`: `Tj = (0.02/20)^{1/3} = 0.1`.
- Branch thresholds for `(1, 2, 10)`: step 1 iff `h ≥ vMax·Ta = 0.7`; step 2 iff `h ≥ 2·aMax³/jMax² = 0.16`.
- Synchronization: replanning with `(vMax/λ, aMax/λ², jMax/λ³)` scales `tf` by exactly `λ` in every branch
  (e.g. full case, `λ = 2` ⇒ `11.4`).

## Edge cases

- `jMax → ∞` ⇒ jerk phases vanish, profile degenerates to a trapezoid (cross-check limit).
- `h` exactly at a branch threshold (0.7 or 0.16 above) ⇒ neighbouring branches give the same `tf`.
- `d == 0` ⇒ zero-length profile; `Sample(0)` is the start, fully at rest.
