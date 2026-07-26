# Continuum Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestContinuum : public ::testing::Test:
    # single unit-length section, straight by default
    std::array<ArcParameters<float>, 1> sec = { { kappa=0, phi=0, length=1 } }
    ContinuumKinematics<float, 1> ck{ sec }
# each case below is a TEST_F(TestContinuum, <name>)
```

## Test cases (Arrange / Act / Assert)

```
straight_section_is_pure_translation:
    Act:    T = ck.SectionTransform({0, 0, 1})
    Assert: T ≈ SE3(I, (0, 0, 1))

quarter_circle_bend:
    Arrange: κ = π/2, length = 1  ⇒ θ = π/2, radius = 2/π
    Assert:  tip position ≈ (2/π)·(1, 0, 1)

bending_plane_angle_rotates_tip:
    Arrange: same κ, s; φ = π/2
    Assert:  tip bends in the y-z plane instead of the x-z plane

forward_composes_sections:
    Arrange: two identical bent sections
    Assert:  Forward = SectionTransform ∘ SectionTransform

inverse_recovers_arc_parameters:
    Arrange: known (κ, φ, s); pose = SectionTransform
    Act:     arc = ck.InverseSection(pose)
    Assert:  arc ≈ (κ, φ, s)

curvature_zero_series_no_blowup:
    Arrange: κ = 1e-9
    Assert:  SectionTransform finite ≈ straight (no 1/κ overflow)

arc_length_preserved:
    Assert: geodesic length of the section == s regardless of κ

tip_orientation_matches_bend:
    Assert: section R rotates the tangent by θ = κ·s
```

## Reference vectors

- Straight unit section ⇒ tip `(0, 0, 1)`, `R = I`.
- `κ = π/2`, `s = 1`, `φ = 0` ⇒ `θ = π/2`, radius `2/π`, tip `= (2/π)·(1, 0, 1)`.

## Edge cases

- `κ → 0` ⇒ series fallback; `φ` undefined but the result is independent of `φ`.
- Full loop `θ = 2π` ⇒ tip returns near the base (closed circle).
- Negative `κ` ⇒ bends the opposite way; sign consistent with `φ`.
- Multi-section inverse ⇒ falls back to iteration (documented, not closed-form).
