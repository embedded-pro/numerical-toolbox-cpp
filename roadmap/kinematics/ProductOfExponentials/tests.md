# Product of Exponentials — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPoE : public ::testing::Test:
    # 2-link unit planar arm as screws about ẑ at x = 0 and x = 1
    std::array<Vector<float,6>, 2> screws = { (ẑ; 0,0,0), (ẑ; 0,-1,0) }
    SE3<float> home = SE3(I, (2,0,0))
    ProductOfExponentials<float, 2> poe{ screws, home }
# each case below is a TEST_F(TestPoE, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_config_returns_home:
    Assert: poe.Compute({0, 0}) ≈ home  (tip at (2,0,0))

single_joint_rotation:
    Act:    T = poe.Compute({π/2, 0})
    Assert: T.p ≈ (0, 2, 0)

elbow_rotation_matches_dh:
    Act:    T = poe.Compute({0, π/2})
    Assert: T.p ≈ (1, 1, 0)

agrees_with_denavit_hartenberg:
    Assert: poe.Compute(q) ≈ dh.Forward(q) over a sweep of q  (same arm)

space_jacobian_first_column_is_screw:
    Assert: SpaceJacobian(q) column 0 == screws[0]

jacobian_matches_finite_difference:
    Assert: SpaceJacobian(q)·q̇ ≈ twist from (Compute(q + εq̇) ⊖ Compute(q)) / ε

prismatic_screw_translates:
    Arrange: screw with ω = 0, v = x̂
    Assert:  Compute({s}) translates the tool by s·x̂

exp_zero_angle_is_identity:
    Assert: SE3::Exp(S, 0) ≈ Identity for each screw
```

## Reference vectors

- 2-link unit arm, `q = (0, π/2)` ⇒ tip `(1, 1, 0)` (identical to the DH result).
- `q = (π/2, 0)` ⇒ tip `(0, 2, 0)`.

## Edge cases

- `q → 0` per joint ⇒ series fallback in `Exp`, no `sinθ/θ` blow-up.
- Prismatic screw (`ω = 0`) ⇒ pure translation, `Exp` skips the rotation branch.
- Non-unit `ω` fed in ⇒ document the normalization expectation.
- Long chain ⇒ compose order fixed (space form left-to-right) to match `M`.
