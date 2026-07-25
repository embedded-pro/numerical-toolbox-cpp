# Pose Inverse Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPoseIk : public ::testing::Test:
    DenavitHartenberg<float, 6>      arm{ ... }       # 6R with spherical wrist
    SpatialJacobian<float, 6>        jac{ arm }
    PoseInverseKinematics<float, 6>  ik{ arm, { λ=0.05, tol=1e-4, 100 } }
# each case below is a TEST_F(TestPoseIk, <name>)
```

## Test cases (Arrange / Act / Assert)

```
round_trip_reachable_pose:
    Arrange: q_true random; target = FK(q_true)
    Act:     r = ik.Solve(target, q_true + small_perturbation)
    Assert:  r.converged; FK(r.q) ≈ target (position + orientation)

position_and_orientation_both_met:
    Assert: PoseError(target, FK(r.q)) < tol in all 6 components

converges_from_nearby_seed:
    Assert: iterations small when q0 near solution (warm start)

pure_orientation_change:
    Arrange: target = same position, rotated tool
    Assert:  solver reorients without large base-joint motion

damping_survives_singularity:
    Arrange: target forcing a wrist singularity
    Assert:  Δq stays bounded; no NaN / Inf

unreachable_target_fails_gracefully:
    Arrange: target beyond arm reach
    Assert:  r.converged == false, finalError > tol, q finite

orientation_error_takes_short_path:
    Arrange: target rotation of +190° about ẑ
    Assert:  ‖e_ω‖ ≈ 170° (short way), not 190°

zero_error_is_immediate:
    Arrange: target = FK(q0)
    Assert:  iterations == 0, converged
```

## Reference vectors

- Reachable `target = FK(q_true)` ⇒ recovered `q` with `‖e‖ < 10⁻⁴`.
- Orientation error of `R` vs `R·Rot(ẑ, 190°)` ⇒ `‖e_ω‖ ≈ 170° = 2.967 rad ≤ π`.

## Edge cases

- Wrist singularity (aligned axes) ⇒ damping keeps the solve finite.
- Target exactly at reach boundary ⇒ slow but bounded convergence.
- Antipodal orientation (180°) ⇒ axis ambiguous; document the tie-break.
- Quaternion double cover ⇒ `q` and `−q` give the same error direction.
