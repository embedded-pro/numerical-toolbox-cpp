# Point-Set Alignment — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestPointSetAlignment : public ::testing::Test:
    std::array<Vector<float, 2>, 4> planar{ (0, 0), (1, 0), (0, 2), (−1, 1) }
    std::array<Vector<float, 3>, 5> spatial{ (0, 0, 0), (1, 0, 0), (0, 2, 0), (0, 0, 3), (1, 1, 1) }
    Matrix3 rotation{ RotationAboutAxis((1, 2, 3)/‖·‖, 0.7) }       # Geometry3D
    Vector3 offset{ 0.2, −0.4, 1.0 }
# each case below is a TEST_F(TestPointSetAlignment, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
planar_rigid_motion_is_recovered_exactly:
    Arrange: target = Rot(30°)·planar + (0.5, −1)
    Assert:  angle ≈ π/6, translation ≈ (0.5, −1), rms ≈ 0
spatial_rigid_motion_is_recovered_exactly:
    Arrange: target = rotation·spatial + offset
    Assert:  rotation ≈ the fixture rotation (1e-5), translation ≈ offset, rms ≈ 0
umeyama_recovers_the_scale:
    Arrange: target = 2.5·rotation·spatial + offset, estimateScale = true
    Assert:  scale ≈ 2.5, translation ≈ offset, rms ≈ 0
mirrored_points_still_give_a_proper_rotation:
    Arrange: target = spatial with z negated
    Assert:  det(rotation) ≈ +1, rms ≈ 0.925196     # a reflection would fit with rms 0
zero_weight_removes_an_outlier:
    Arrange: target = rotation·spatial + offset, target[4] += (5, 5, 5), weights { 1, 1, 1, 1, 0 }
    Assert:  rotation ≈ fixture rotation, rms ≈ 0   # with unit weights rms = 3.442415
noisy_planar_fit_matches_the_reference:
    Arrange: target as in the first case plus noise { (0.01, −0.02), (0, 0.015), (−0.01, 0), (0.02, 0.01) }
    Assert:  angle ≈ 0.528695, translation ≈ (0.508305, −0.996831), rms ≈ 0.016549
degenerate_inputs_are_rejected:
    Assert: collinear { (0,0,0), (1,1,1), (2,2,2) } ⇒ nullopt;  one 2-D point ⇒ nullopt;
            size mismatch ⇒ nullopt
```

## Reference vectors

- Fixture rotation `Rot((1,2,3)/√14, 0.7)` =
  `[[0.781639, −0.482929, 0.394740], [0.550117, 0.832030, −0.071392], [−0.293958, 0.272956, 0.916015]]`.
- Mirrored `spatial` (z → −z): best proper rotation leaves `rms = 0.925196`.
- Collinear set: singular values `(2, 0, 0)` ⇒ rejected.

## Edge cases

- Exactly `Dim` non-degenerate points (2 in 2-D, 3 non-collinear in 3-D) ⇒ exact solution.
- Translated clouds far from the origin (offset `10³`) keep `rms ≈ 0` to `1e-3`, thanks to centring.
