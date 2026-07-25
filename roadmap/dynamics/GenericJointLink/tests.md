# Generic Joint Link — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestGenericJointLink : public ::testing::Test:
    # unit axes for readable expectations
    Vector3 z = {0, 0, 1}
    Vector3 x = {1, 0, 0}
    GenericJointLink<float> MakeRevolute(axis, parentToJoint)
    GenericJointLink<float> MakePrismatic(axis, parentToJoint)
# each case below is a TEST_F(TestGenericJointLink, <name>)
```

## Test cases (Arrange / Act / Assert)

```
revolute_rotates_about_axis:
    Arrange: revolute about z, q = pi/2
    Act:     (R, p) = JointTransform(q)
    Assert:  R maps x -> y (±tol), p == parentToJoint

revolute_zero_angle_is_identity:
    Arrange: revolute about z, q = 0
    Assert:  R == I, p == parentToJoint

prismatic_translates_along_axis:
    Arrange: prismatic along x, parentToJoint = {0,0,0}, q = 0.3
    Act:     (R, p) = JointTransform(q)
    Assert:  R == I, p == {0.3, 0, 0}

prismatic_keeps_orientation:
    Arrange: prismatic along z, sweep q
    Assert:  R == I for all q (no rotation)

revolute_motion_subspace:
    Arrange: revolute about axis a
    Assert:  AngularAxis() == a, LinearAxis() == 0

prismatic_motion_subspace:
    Arrange: prismatic along axis a
    Assert:  LinearAxis() == a, AngularAxis() == 0

from_revolute_matches_legacy:
    Arrange: RevoluteJointLink r; g = FromRevolute(r)
    Assert:  same mass/inertia/axis; JointTransform is a pure rotation

offset_joint_origin_applied:
    Arrange: prismatic along x, parentToJoint = {0,1,0}, q = 0.5
    Assert:  p == {0.5, 1, 0}
```

## Reference vectors

- Revolute z, q = π/2: `R·[1,0,0]ᵀ = [0,1,0]ᵀ`.
- Prismatic x, q = d: `p = parentToJoint + [d,0,0]ᵀ`, `R = I`.

## Edge cases

- Negative `q` (reverse rotation / retracting slide) mirrors the positive case.
- Non-unit `axis` — assert construction normalizes (or a guard rejects it).
- Full `2π` revolute wrap returns to identity within tolerance.
