# Denavit-Hartenberg Parameters — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestDenavitHartenberg : public ::testing::Test:
    # 2-link planar arm: a₁ = a₂ = 1, α = d = 0, both revolute
    std::array<DhLink<float>, 2> planar = { {1,0,0,0,Revolute}, {1,0,0,0,Revolute} }
    DenavitHartenberg<float, 2>  dh{ planar }
# each case below is a TEST_F(TestDenavitHartenberg, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_config_is_stretched_along_x:
    Act:    T = dh.Forward({0, 0})
    Assert: T.p ≈ (2, 0, 0),  T.R ≈ Identity

single_link_revolute_rotates:
    Arrange: 1-link arm a = 1
    Act:     T = dh.Forward({π/2})
    Assert:  T.p ≈ (0, 1, 0)

planar_elbow_ninety_deg:
    Act:    T = dh.Forward({0, π/2})
    Assert: T.p ≈ (1, 1, 0)

prismatic_link_translates_along_z:
    Arrange: single Prismatic link, a = α = θ = 0
    Act:     T = dh.Forward({0.5})
    Assert:  T.p ≈ (0, 0, 0.5)

link_twist_alpha_reorients_frame:
    Arrange: link a = 0, α = π/2
    Assert:  LinkTransform.R maps ẑ → -ŷ  (axis reorientation)

frame_chain_last_matches_forward:
    Assert: FrameChain(q).back() ≈ Forward(q)

link_transform_is_orthonormal:
    Assert: RᵀR ≈ Identity for arbitrary q

modified_vs_standard_differ:
    Assert: modified-DH Forward ≠ standard Forward for the same table (documented)
```

## Reference vectors

- 2-link unit planar arm, `q = (0, π/2)` ⇒ tip `(1, 1, 0)`.
- Single revolute `a = 1`, `q = π/2` ⇒ tip `(0, 1, 0)`.
- Prismatic `d = 0.5` ⇒ tip `(0, 0, 0.5)`.

## Edge cases

- `alpha = ±π/2` ⇒ frame axis swaps; verify no sign error in `sa`.
- Zero-length links (`a = d = 0`) ⇒ pure rotation joints.
- Mixed revolute/prismatic chain ⇒ correct variable substitution per link.
- Full-turn `theta = 2π` ⇒ same pose as `theta = 0` (wrap-agnostic).
