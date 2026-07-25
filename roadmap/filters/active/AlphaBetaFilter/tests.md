# Alpha-Beta / Alpha-Beta-Gamma Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestAlphaBetaFilter : public ::testing::Test:
    AlphaBetaFilter<float, 2> filter{ 0.5f, 0.1f, 1.0f }   # alpha, beta, Ts
# each case below is a TEST_F(TestAlphaBetaFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
first_sample_seeds_position:
    Arrange: fresh filter
    Act:     y = Filter(0.4)
    Assert:  y == 0.4 and velocity == 0

constant_position_zero_velocity:
    Arrange: feed constant z = 0.3 for 50 samples
    Assert:  position -> 0.3, velocity -> 0  (steady state, no lag)

constant_velocity_tracks_ramp:
    Arrange: z[n] = v0 * n * Ts (noise-free ramp), v0 = 0.2
    Assert:  after settling, velocity ≈ v0 and position lag ≈ 0

step_response_settles:
    Arrange: step z from 0 -> 1
    Assert:  position converges to 1, monotone within stability bounds

gains_from_tracking_index_are_stable:
    Arrange: g = GainsFromTrackingIndex(0.5)
    Assert:  0 < g.alpha < 1 and 0 < g.beta < 4 - 2*g.alpha

noise_is_attenuated:
    Arrange: constant true position + zero-mean noise
    Assert:  variance(output) < variance(input)

reset_clears_state:
    Arrange: run samples, Reset(0)
    Assert:  next Filter(z) seeds position = z, velocity = 0

alpha_beta_gamma_tracks_acceleration:   # AlphaBetaFilter<float,3> fixture
    Arrange: z[n] = 0.5*a0*(n*Ts)^2 (constant-accel arc)
    Assert:  acceleration estimate ≈ a0, bounded position lag
```

## Reference vectors

- Constant-velocity ramp: steady-state velocity estimate equals the true slope (zero lag).
- α-β Kalata design: `lambda = 1` ⇒ published `alpha`, `beta` pair reproduced by the helper.

## Edge cases

- `Ts` very small ⇒ `beta/Ts`, `2*gamma/Ts²` grow large: assert the estimate stays bounded.
- Gains at stability boundary (`beta -> 4 - 2*alpha`) ⇒ marginally stable, must not diverge.
- Order-3 filter fed a pure ramp ⇒ acceleration estimate stays ≈ 0 (no phantom accel).
