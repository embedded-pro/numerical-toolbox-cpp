# SE(2) Transform — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSE2Transform : public ::testing::Test:
    SE2Transform<float> a{ FromPose(1, 2, π/2) }
    SE2Transform<float> b{ FromPose(−0.5, 0.3, −π/3) }
    Vector3<float> twist{ 0.4, −0.2, 0.7 }
    tolerance 1e-5
# each case below is a TEST_F(TestSE2Transform, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
apply_rotates_then_translates:
    Assert: a.Apply((1, 0)) ≈ (1, 3)
composition_applies_right_operand_first:
    Assert: (a * b).Apply(x) ≈ a.Apply(b.Apply(x)) for x = (0.3, −0.2);  (a * b).Pose() ≈ (0.7, 1.5, π/6)
inverse_composes_to_identity:
    Assert: a.Inverse().Pose() ≈ (−2, 1, −π/2);  a * a.Inverse() ≈ Identity  and  a.Inverse() * a ≈ Identity
pose_returns_wrapped_heading:
    Assert: FromPose(0, 0, 3π/2).Heading() ≈ −π/2;  b.Pose() ≈ (−0.5, 0.3, −π/3)
homogeneous_matches_block_form:
    Assert: a.Homogeneous() == [[0, −1, 1], [1, 0, 2], [0, 0, 1]]
adjoint_maps_twists_consistently_with_composition:
    Assert: Exp(a.TransformTwist(twist), t) ≈ a * Exp(twist, t) * a.Inverse()  for t = 0.4 and t = 2
adjoint_of_product_is_product_of_adjoints:
    Assert: (a * b).Adjoint() ≈ a.Adjoint() · b.Adjoint()
wrench_and_twist_power_is_frame_invariant:
    Arrange: wrench W = (1.5, −0.7, 0.3)
    Assert:  a.TransformWrench(W) · a.TransformTwist(twist) ≈ W · twist = 0.95
exp_of_unicycle_arc:
    Assert: Exp((1, 0, 1), π/2).Pose() ≈ (1, 1, π/2)       # quarter circle of radius 1
exp_of_pure_translation:
    Assert: Exp((0.3, −0.4, 0), 2).Pose() ≈ (0.6, −0.8, 0)
exp_is_continuous_across_series_threshold:
    Arrange: ξ = (0.8, 0.3, φ), θ = 1, φ ∈ {0.1 − 1e-4, 0.1 + 1e-4}
    Assert:  both translations agree with the closed form V(φ)·(0.8, 0.3) to 1e-6
log_inverts_exp:
    Assert: a.Log() ≈ (3π/4, π/4, π/2);  Exp(T.Log(), 1) ≈ T for T = a, b, FromPose(0.2, −0.4, 179.9°)
log_at_half_turn_is_finite:
    Assert: FromPose(1, 0, π).Log() ≈ (0, −π/2, π)
pose_error_takes_the_short_way:
    Arrange: current = FromPose(1, 1, 0.2), target = FromPose(2, 0.5, 0.2 + 190°)
    Assert:  PoseError(target, current) ≈ (1, −0.5, −170°)   # −2.967060 rad
normalized_restores_unit_rotation:
    Arrange: T = { 1.001·cos 0.3, 1.001·sin 0.3, (1, 2) }
    Assert:  T.Normalized() has cos² + sin² ≈ 1, heading 0.3, translation (1, 2)
to_se3_rotates_about_z:
    Assert: a.ToSE3() ≈ { RotationAboutAxis(ẑ, π/2), (1, 2, 0) }
```

## Reference vectors

- `FromPose(1, 2, π/2)` applied to `(1, 0)` ⇒ `(1, 3)`; its inverse is `(−2, 1, −π/2)`.
- `a * b` ⇒ `(0.7, 1.5, π/6)`.
- `a.Log()` ⇒ `(3π/4, π/4, π/2) = (2.356194, 0.785398, 1.570796)`; `b.Log()` ⇒ `(−0.610529, 0.010271, −π/3)`.
- `Exp` agrees with a double-precision 3×3 matrix exponential of `[[0, −ω, vx], [ω, 0, vy], [0, 0, 0]]·θ`
  to `8e-10` over 2000 random twists; `Exp(Log(T))` round-trips to `6e-10`.

## Edge cases

- `ω = 0` (pure translation) and `|ω·θ| > π` in `Exp` (the result is still exact; only `Log` wraps).
- `Log` at heading exactly `π`: the branch returns `+π`, never `−π`.
- 10⁵ compositions of a small step with `Normalized` after each keep `cos² + sin² = 1` to `1e-6`.
