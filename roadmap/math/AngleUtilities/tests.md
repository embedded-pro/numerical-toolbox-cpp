# Angle Utilities — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestAngleUtilities : public ::testing::Test:
    float pi = std::numbers::pi_v<float>
# each case below is a TEST_F(TestAngleUtilities, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
wrap_to_pi_maps_into_half_open_interval:
    Assert: WrapToPi(3π/2) ≈ −π/2;  WrapToPi(−3π/2) ≈ π/2;  WrapToPi(0) == 0;  WrapToPi(7π/3) ≈ π/3
wrap_to_pi_sends_both_half_turns_to_plus_pi:
    Assert: WrapToPi(π) ≈ π  and  WrapToPi(−π) ≈ π
wrap_to_two_pi_maps_into_zero_to_two_pi:
    Assert: WrapToTwoPi(−π/2) ≈ 3π/2;  WrapToTwoPi(2π) ≈ 0;  WrapToTwoPi(5π) ≈ π
angle_difference_takes_the_short_way:
    Assert: AngleDifference(170°, −170°) ≈ −20°  and  AngleDifference(−170°, 170°) ≈ 20°
unwrap_follows_a_continuous_rotation:
    Arrange: true angle θₖ = 0.3·k rad, k = 0..40 (≈ 2 turns); wrapped = WrapToPi(θₖ)
    Act:     u₀ = wrapped₀;  uₖ = UnwrapAngle(uₖ₋₁, wrappedₖ)
    Assert:  uₖ ≈ θₖ for every k
```

## Reference vectors

- `7π/3 = 2π + π/3` ⇒ `π/3`; `5π = 2·2π + π` ⇒ `π` in `[0, 2π)`.
- `170° − (−170°) = 340° ≡ −20°`.

## Edge cases

- `±π` exactly ⇒ `+π` (normative convention).
- `|angle|` ≈ `1e4` rad ⇒ result within `|angle|·ε` of the exact wrap; assert with that tolerance.
- `WrapToTwoPi` of a value just below `0` ⇒ never returns `2π`.
