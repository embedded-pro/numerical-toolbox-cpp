# Manipulability Index — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestManipulability : public ::testing::Test:
    DenavitHartenberg<float, 2>   arm{ ... }      # 2-link unit planar arm
    SpatialJacobian<float, 2>     jac{ arm }
    ManipulabilityIndex<float, 2> w{ jac }
# each case below is a TEST_F(TestManipulability, <name>)
```

## Test cases (Arrange / Act / Assert)

```
right_angle_elbow_is_most_dexterous:
    Act:    value = w.Compute({0, π/2})
    Assert: value is the local maximum over q₂

stretched_arm_is_singular:
    Act:    value = w.Compute({0, 0})
    Assert: value ≈ 0  (elbow straight ⇒ lost a DOF)

folded_arm_is_singular:
    Assert: w.Compute({0, π}) ≈ 0

index_is_nonnegative:
    Assert: Compute(q) ≥ 0 across a sweep of q

near_singular_flag_trips:
    Arrange: q close to stretched
    Assert:  NearSingular(q, eps) == true

condition_number_grows_near_singularity:
    Assert: ConditionNumber → large as q → stretched

ellipsoid_axes_match_singular_values:
    Assert: EllipsoidAxes(q) are sorted σ; their product ≈ Compute(q)

symmetry_about_elbow_sign:
    Assert: Compute({0, +β}) ≈ Compute({0, -β})
```

## Reference vectors

- 2-link unit arm: `w(q₂) = |sin q₂|` (planar case) ⇒ max at `q₂ = π/2`, zero at `q₂ ∈ {0, π}`.
- Condition number `→ ∞` as `q₂ → 0`.

## Edge cases

- Square arm (`N = 6`) ⇒ `w = |det J|`, skip the Gram product.
- Redundant arm (`N > 6`) ⇒ `√det(J Jᵀ)` only; `det J` undefined.
- Exactly singular `J` ⇒ `ConditionNumber` division guarded (returns ∞ sentinel).
- Tiny `w` below `float` epsilon ⇒ `NearSingular` still deterministic.
