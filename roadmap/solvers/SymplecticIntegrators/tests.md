# Symplectic Integrators — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSymplecticIntegrators : public ::testing::Test:
    HarmonicOscillator oscillator            # q̈ = −ω²q + u, ω = 1 (concrete SecondOrderSystem)
    StrictMock<SecondOrderSystemMock<float, 1, 1>> system
    SemiImplicitEuler<float, 1, 1> euler{ oscillator, 0.01f }
    VelocityVerlet<float, 1, 1>    verlet{ oscillator, 0.01f }
# each case below is a TEST_F(TestSymplecticIntegrators, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
semi_implicit_euler_updates_position_with_new_velocity:
    Arrange: EXPECT_CALL(system, Acceleration(q = 1, q̇ = 0, u = 0)) returns 2; h = 0.1
    Assert:  Step({1, 0}, 0) == { 1 + 0.1·0.2, 0.2 }  (= { 1.02, 0.2 })
velocity_verlet_evaluates_at_half_step_velocity:
    Arrange: EXPECT_CALL sequence: Acceleration(1, 0, 0) → 2, then Acceleration(1.01, 0.1, 0) → 2; h = 0.1
    Assert:  Step({1, 0}, 0) == { 1.01, 0.2 }
velocity_verlet_is_exact_for_constant_acceleration:
    Arrange: oscillator with ω = 0 and u = 3, 100 steps from rest
    Assert:  q ≈ ½·3·t² and q̇ ≈ 3·t at t = 1
energy_error_stays_bounded_over_long_runs:
    Arrange: oscillator from q = 1, q̇ = 0; 10⁵ steps (≈ 160 periods) with each method
    Assert:  |E − E₀| / E₀ stays < 0.6 % (Euler) and < 1e-4 (Verlet) for every step
verlet_is_second_order:
    Arrange: q(1) after 1 s with h and h/2
    Assert:  error(h) / error(h/2) ≈ 4 (±10 %)
```

## Reference vectors

- Harmonic oscillator `q(t) = cos t`, energy `E = ½(q̇² + q²) = ½`.
- Semi-implicit Euler conserves a modified energy `½(q̇² + q² − h·q·q̇)`: the true energy oscillates with
  relative amplitude ≈ `h·ω/2` (0.5 % at `h = 0.01`) and never drifts.

## Edge cases

- `h·ω` near `2` ⇒ still bounded just below, divergent above (documented stability limit).
- Zero acceleration ⇒ uniform motion `q + h·q̇` exactly.
