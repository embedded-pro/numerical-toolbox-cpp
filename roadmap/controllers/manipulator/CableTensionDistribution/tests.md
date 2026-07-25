# Cable Tension Distribution — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCableTensionDistribution : public ::testing::Test:
    # structure Jacobian injected & mocked; planar WrenchDim = 2, NumCables = 3:
    StrictMock<kinematics::MockSpatialJacobian<float,3>> structure
    float tMin = 10
    float tMax = 200
    CableTensionDistribution<float,3,2> distributor{ structure, tMin, tMax }
    # concrete Mpc QP solver used internally (not mocked)
# each case below is a TEST_F(TestCableTensionDistribution, <name>)
```

## Test cases (Arrange / Act / Assert)

```
square_case_unique_solution:
    Arrange: NumCables == WrenchDim, invertible A (mock structure)
    Act:     t = Distribute(wDesired, pose)
    Assert:  t == A⁻¹ · wDesired

redundant_case_centres_tensions:
    Arrange: A = 2×3, one-DOF redundancy
    Assert:  t is the min-norm-to-tMid solution satisfying A·t = wDesired

all_tensions_nonnegative:
    Arrange: any feasible wrench
    Assert:  t[i] >= tMin > 0 for all i

respects_upper_bound:
    Arrange: large wrench pushing one cable toward the limit
    Assert:  t[i] <= tMax for all i

wrench_is_reproduced:
    Arrange: feasible wrench
    Assert:  A · t ≈ wDesired  (equality constraint satisfied)

infeasible_pose_returns_nullopt:
    Arrange: wrench outside the feasible cone (no pull-only solution)
    Assert:  Distribute(...) == nullopt

zero_wrench_uses_internal_pretension:
    Arrange: wDesired = 0
    Assert:  t in null(A), all >= tMin (taut, no external load)

structure_matrix_queried_once:
    Arrange: any pose
    Assert:  structure.Compute called exactly once (StrictMock)
```

## Reference vectors

- Planar 3-cable point mass, symmetric angles: hand-computed `A` (2×3); a vertical wrench ⇒ symmetric
  tensions, exactly reproducible.
- Square `A` (NumCables = WrenchDim): unique `t = A⁻¹w` — golden check.

## Edge cases

- Wrench on the workspace boundary: one tension saturates at `tMax`, still feasible.
- Beyond the boundary: `nullopt`.
- Near-parallel cables (`A` ill-conditioned): QP regularisation / `tMid` centring keeps `t` finite.
