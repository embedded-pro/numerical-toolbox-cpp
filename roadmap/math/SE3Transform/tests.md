# SE(3) Transform — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSE3 : public ::testing::Test:
    SE3<float> identity
    SE3<float> gAB   # rotation 90° about ẑ + translation (1,0,0)
# each case below is a TEST_F(TestSE3, <name>)
```

## Test cases (Arrange / Act / Assert)

```
identity_apply_is_noop:
    Assert: identity.Apply(p) == p

compose_with_inverse_is_identity:
    Arrange: g = gAB
    Assert:  g * g.Inverse() ≈ Identity

apply_rotates_then_translates:
    Arrange: g = rot(ẑ, 90°) + p = (1,0,0)
    Assert:  g.Apply((1,0,0)) ≈ (1,1,0)

inverse_matches_transpose_form:
    Assert: Inverse().R == Rᵀ  and  Inverse().p == -Rᵀp

homogeneous_matches_block_form:
    Assert: Homogeneous() == [[R, p],[0, 0, 0, 1]]

adjoint_transforms_twist:
    Arrange: known twist in frame B
    Assert:  Adjoint()·ξ matches the hand-computed twist in frame A

exp_pure_translation:
    Arrange: ω = 0, v = (1,0,0), θ = 2
    Assert:  Exp ≈ SE3(I, (2,0,0))

exp_pure_rotation:
    Arrange: ω = ẑ, v = 0, θ = π/2
    Assert:  Exp ≈ SE3(rot(ẑ, 90°), 0)

exp_log_round_trip:
    Arrange: random small twist ξ
    Assert:  Log(Exp(ξ)) ≈ ξ

adjoint_of_product_is_product_of_adjoints:
    Assert: Adjoint(g1*g2) ≈ Adjoint(g1)·Adjoint(g2)
```

## Reference vectors

- `rot(ẑ, 90°) ⊕ (1,0,0)` applied to `(1,0,0)` ⇒ `(1,1,0)`.
- `Exp(ω = ẑ, v = 0, θ = π/2)` ⇒ rotation-only transform `rot(ẑ, 90°)`.

## Edge cases

- `θ → 0` in `Exp`/`Log` ⇒ series fallback, no `sinθ/θ` division blow-up.
- `θ = π` rotation ⇒ axis recovery in `Log` stays well-defined.
- Non-orthonormal `R` fed in ⇒ document the reorthonormalization expectation.
- Twist with zero angular part ⇒ `Adjoint` lower-left `[p]×R` still correct.
