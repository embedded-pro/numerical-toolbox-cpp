# Parallel Manipulator Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestStewartGough : public ::testing::Test:
    PlatformGeometry<float, 6>  hexapod{ ... }        # symmetric 6-6 platform
    ParallelManipulatorKinematics<float, 6> kin{ hexapod, { 1e-6, 50 } }
# each case below is a TEST_F(TestStewartGough, <name>)
```

## Test cases (Arrange / Act / Assert)

```
inverse_of_home_pose:
    Act:    L = kin.Inverse(homePose)
    Assert: all legs equal the nominal home length

inverse_is_closed_form_deterministic:
    Assert: Inverse(pose) is repeatable, no iteration state

forward_inverse_round_trip:
    Arrange: pose_true; L = kin.Inverse(pose_true)
    Act:     pose = kin.Forward(L, pose_true + small_perturbation)
    Assert:  pose ≈ pose_true

forward_converges_from_nearby_guess:
    Assert: Newton reaches tolerance in few iterations (warm start)

leg_length_monotone_with_height:
    Arrange: raise the platform along +z
    Assert:  each leg length increases

assembly_mode_selected_by_guess:
    Arrange: two guesses bracketing different modes
    Assert:  Forward returns the pose near each guess

singular_pose_illconditioned:
    Arrange: geometry at a platform singularity
    Assert:  Forward step damped; no NaN

delta_closed_form_forward:
    Arrange: 3-leg Delta geometry
    Assert:  closed-form Forward matches the Inverse round-trip
```

## Reference vectors

- Home pose ⇒ all six legs at nominal length `L₀`.
- Pure `+z` translation `Δh` ⇒ symmetric increase in every leg length.

## Edge cases

- Platform singularity ⇒ leg Jacobian rank-deficient; damped fallback.
- Wrong assembly-mode guess ⇒ converges to a different valid pose (documented).
- Unreachable leg lengths ⇒ Forward fails gracefully, returns best effort.
- Delta (3 legs) vs Stewart (6 legs) ⇒ different `NumLegs` template instantiations.
