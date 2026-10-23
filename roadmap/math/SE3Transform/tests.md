# SE(3) Transform — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSE3Transform : public ::testing::Test:
    SE3Transform<float> a{ RotationAboutAxis(ẑ, π/2), (1, 0, 0) }
    SE3Transform<float> b{ RotationAboutAxis(ŷ, π/3), (0, 2, 0) }
# each case below is a TEST_F(TestSE3Transform, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
apply_rotates_then_translates:
    Assert: a.Apply((1, 0, 0)) ≈ (1, 1, 0)
composition_applies_right_operand_first:
    Assert: (a * b).Apply(x) ≈ a.Apply(b.Apply(x)) for x = (0.3, −0.2, 0.5)
inverse_composes_to_identity:
    Assert: a * a.Inverse() ≈ Identity  and  a.Inverse() * a ≈ Identity
homogeneous_matches_block_form:
    Assert: a.Homogeneous() == [[R, p],[0, 0, 0, 1]]
adjoint_maps_twists_consistently_with_composition:
    Arrange: twist V in the frame of a
    Assert:  Exp(a.TransformTwist(V), t) ≈ a * Exp(V, t) * a.Inverse()  (t = 0.4)
adjoint_of_product_is_product_of_adjoints:
    Assert: (a * b).Adjoint() ≈ a.Adjoint() · b.Adjoint()
wrench_and_twist_power_is_frame_invariant:
    Arrange: twist V, wrench W in the same frame
    Assert:  a.TransformWrench(W) · a.TransformTwist(V) ≈ W · V
exp_of_pure_rotation_about_offset_axis:
    Arrange: screw about ẑ through (1, 0, 0): ξ = (−ẑ × (1,0,0); ẑ) = ((0,−1,0); ẑ)
    Assert:  Exp(ξ, π/2).p ≈ (1, −1, 0) and R ≈ Rz(π/2)
exp_of_pure_translation:
    Assert: Exp(((1,0,0); 0), 0.3) ≈ { I, (0.3, 0, 0) }
log_inverts_exp:
    Assert: Exp(Log(T), 1) ≈ T for T = a, b, and a rotation of 179.9°
log_near_zero_and_near_pi_stays_finite:
    Assert: LogRotation of rotations of 1e-6 rad and π − 1e-4 rad are finite with the expected angle
pose_error_takes_the_short_way:
    Arrange: target = current rotated by +190° about ẑ
    Assert:  ‖PoseError(target, current).angular‖ ≈ 170° (2.967 rad), direction −ẑ
```

## Reference vectors

- `Rz(π/2) ⊕ (1,0,0)` applied to `(1,0,0)` ⇒ `(1,1,0)`.
- Screw about `ẑ` through `(1,0,0)` by `π/2` maps the origin to `(1,−1,0)`.
- Rotation error of `+190°` about `ẑ` ⇒ rotation vector `−170°·ẑ`.

## Edge cases

- `Exp` with `ω = 0` (pure translation) and with non-unit `ω` (magnitude folded into the angle).
- `LogRotation` at `φ = π` exactly: either axis sign is valid; the test accepts both.
- Repeated composition in `float` keeps `RᵀR ≈ I` to `1e-5` after 1000 random products.
