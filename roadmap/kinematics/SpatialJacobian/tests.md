# Spatial Jacobian — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSpatialJacobian : public ::testing::Test:
    # 2-link unit planar arm, both revolute, in the x-y plane
    DenavitHartenberg<float, 2>  arm{ ... }
    SpatialJacobian<float, 2>    jac{ arm }
# each case below is a TEST_F(TestSpatialJacobian, <name>)
```

## Test cases (Arrange / Act / Assert)

```
planar_arm_column_dimensions:
    Act:    J = jac.Compute({0, 0})
    Assert: J is 6×2

stretched_arm_linear_block:
    Arrange: q = (0, 0), tip at (2,0,0)
    Act:     J = jac.Compute(q)
    Assert:  Jv column0 ≈ (0, 2, 0),  Jv column1 ≈ (0, 1, 0)

revolute_angular_block_is_axis:
    Assert: Jw columns ≈ ẑ for both joints (rotation about z)

twist_matches_finite_difference:
    Arrange: q̇ = (1, 0)
    Assert:  EndEffectorTwist ≈ (FK(q + εq̇) - FK(q)) / ε

transpose_maps_wrench_to_torque:
    Arrange: unit force f = x̂ at the tip
    Assert:  JointTorques ≈ hand-computed moment arms

prismatic_column_is_pure_translation:
    Arrange: chain with a prismatic joint
    Assert:  that column = (axis; 0)

singular_configuration_drops_rank:
    Arrange: fully stretched arm (q = (0,0))
    Assert:  columns linearly dependent ⇒ rank < 2

zero_config_is_deterministic:
    Assert: Compute is repeatable and side-effect free
```

## Reference vectors

- 2-link unit arm at `q = (0,0)`: `Jv = [[0,0],[2,1],[0,0]]`, `Jw = [[0,0],[0,0],[1,1]]`.
- Unit tip force `x̂` on that arm ⇒ joint torques `(0, 0)` (force along the link line).

## Edge cases

- Fully stretched / folded arm ⇒ rank-deficient Jacobian (singularity).
- Prismatic-only chain ⇒ zero angular block.
- Single-joint arm ⇒ `6×1` Jacobian, no cross-column coupling.
- Twist ordering `(v; ω)` vs `(ω; v)` ⇒ pin the convention in one assertion.
