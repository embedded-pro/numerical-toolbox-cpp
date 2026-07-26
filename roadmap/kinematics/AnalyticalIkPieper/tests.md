# Analytical IK (Pieper) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPieperIk : public ::testing::Test:
    std::array<DhLink<float>, 6>  puma{ ... }        # PUMA-like 6R, spherical wrist
    AnalyticalIkPieper<float>     ik{ puma }
    DenavitHartenberg<float, 6>   fk{ puma }         # for round-trips
# each case below is a TEST_F(TestPieperIk, <name>)
```

## Test cases (Arrange / Act / Assert)

```
every_solution_reproduces_target:
    Arrange: q_true; target = fk.Forward(q_true)
    Act:     sols = ik.Solve(target)
    Assert:  for each k < count: fk.Forward(sols.q[k]) ≈ target

finds_the_known_configuration:
    Assert: some returned branch ≈ q_true (within angle wrap)

enumerates_up_to_eight:
    Assert: count ≤ 8; a generic reachable pose yields 8

elbow_up_and_down_present:
    Assert: two solutions share θ1 but differ in θ3 sign

unreachable_target_returns_zero:
    Arrange: target far outside the workspace
    Assert:  count == 0

boundary_pose_is_single_elbow:
    Arrange: target at max reach (arm straight)
    Assert:  elbow-up and elbow-down coincide (c3 = 1)

wrist_singularity_flagged:
    Arrange: pose with θ5 = 0
    Assert:  θ4 / θ6 use the documented tie-break; their sum is correct

no_iteration_side_effects:
    Assert: Solve is pure / const; repeated calls identical
```

## Reference vectors

- `target = fk.Forward(q_true)` ⇒ at least one branch matches `q_true`; all `count` branches satisfy FK.
- Fully stretched target ⇒ `c3 = 1`, single elbow solution.

## Edge cases

- `|c3| > 1` ⇒ branch pruned (unreachable).
- Wrist singularity `θ5 = 0` ⇒ `θ4 + θ6` fixed, split by convention.
- Shoulder singularity (`p_wc` on axis 1) ⇒ `θ1` free; documented handling.
- Angle wrap ⇒ compare solutions modulo `2π`.
