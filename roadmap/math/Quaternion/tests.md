# Quaternion — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestQuaternion : public ::testing::Test:
    Quaternion<float> identity = Quaternion<float>::Identity()
# each case below is a TEST_F(TestQuaternion, <name>)
```

## Test cases (Arrange / Act / Assert)

```
identity_is_neutral_product:
    Arrange: q = FromAxisAngle(ẑ, 0.5)
    Act:     r = q * Identity()
    Assert:  r ≈ q

hamilton_product_noncommutative:
    Arrange: qx = rot(x̂, 90°), qy = rot(ŷ, 90°)
    Assert:  qx*qy ≠ qy*qx  (against documented reference values)

conjugate_inverts_unit_rotation:
    Arrange: unit q
    Assert:  q * Conjugate(q) ≈ Identity

rotate_vector_matches_matrix:
    Arrange: q from axis-angle, vector v
    Assert:  q.Rotate(v) ≈ q.ToRotationMatrix() * v

axis_angle_round_trip:
    Arrange: axis ẑ, angle 90°
    Assert:  rotates x̂ -> ŷ  (±tol)

matrix_round_trip:
    Arrange: q -> R -> q'
    Assert:  q' ≈ ±q  (same rotation, double cover)

slerp_endpoints:
    Assert:  Slerp(a, b, 0) ≈ a  and  Slerp(a, b, 1) ≈ b

slerp_midpoint_constant_speed:
    Arrange: a = Identity, b = rot(ẑ, 90°)
    Assert:  Slerp(a, b, 0.5) ≈ rot(ẑ, 45°)

normalize_restores_unit_norm:
    Arrange: scale a unit q by 1.1
    Assert:  Norm() ≈ 1 after Normalize()

euler_round_trip:
    Arrange: roll/pitch/yaw within the non-gimbal range
    Assert:  ToEuler(FromEuler(rpy)) ≈ rpy
```

## Reference vectors

- `rot(ẑ, 90°)` ⇒ `q = (√2/2, 0, 0, √2/2)`, maps `x̂ → ŷ`.
- `Slerp(Identity, rot(ẑ, 90°), 0.5) = rot(ẑ, 45°)`.

## Edge cases

- Antipodal SLERP inputs (`dot < 0`) ⇒ short-arc sign flip.
- Near-parallel inputs (`dot > 0.9995`) ⇒ normalized-lerp fallback (no `acos` blow-up).
- Gimbal-lock pitch = ±90° in `ToEuler` ⇒ documented degenerate handling.
