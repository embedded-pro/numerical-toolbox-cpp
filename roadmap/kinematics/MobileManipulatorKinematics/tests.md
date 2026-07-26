# Mobile-Manipulator Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestMobileManipulator : public ::testing::Test:
    DenavitHartenberg<float, 3>           arm{ ... }
    SpatialJacobian<float, 3>             armJac{ arm }
    MobileManipulatorKinematics<float, 3> mm{ armJac, mount, wheelBase=0.3f }
# each case below is a TEST_F(TestMobileManipulator, <name>)
```

## Test cases (Arrange / Act / Assert)

```
combined_jacobian_dimensions:
    Act:    J = mm.Compute(base0, q0)
    Assert: J is 6 × (2 + 3)

base_pose_composes_with_arm:
    Assert: EndEffectorPose = T_base · mount · FK_arm(q)

nonholonomic_constraint_blocks_sideslip:
    Assert: BaseConstraint(base)·(v,ω) has zero lateral component in the world frame

pure_forward_drive_moves_tool:
    Arrange: (v=1, ω=0), q̇ = 0
    Assert:  tool linear velocity ≈ the forward heading direction

base_yaw_moves_far_tool_more:
    Arrange: ω = 1, large lever arm r
    Assert:  tool linear-velocity magnitude scales with |r|

arm_only_motion_when_base_still:
    Arrange: (v, ω) = 0
    Assert:  tool velocity = armJac·q̇

redundancy_split_base_and_arm:
    Arrange: task rate reachable by base or arm
    Assert:  null-space resolution distributes the motion (M14)

twist_matches_finite_difference:
    Assert: J·[v, ω, q̇] ≈ (pose(t + ε) ⊖ pose(t)) / ε
```

## Reference vectors

- Base heading `φ = 0`, `(v=1, ω=0)` ⇒ tool linear velocity `(1, 0, 0)` plus any arm term.
- Nonholonomic `S(φ)` ⇒ `[-sinφ, cosφ, 0]·q̇_base = 0` for all `(v, ω)`.

## Edge cases

- Zero lever arm (tool over the base axis) ⇒ base yaw adds no tool translation.
- Pure spin (`v=0, ω≠0`) ⇒ tool moves only if `r ≠ 0`.
- Heading wrap (`φ = ±π`) ⇒ `S(φ)` still consistent.
- Non-redundant case (task = base + arm DOF) ⇒ unique split, empty null space.
