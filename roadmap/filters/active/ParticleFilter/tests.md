# Particle Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestParticleFilter : public ::testing::Test:
    ParticleFilter<float, 1, 41> grid      # Initialize(i ⇒ −2 + 4i/40): an evenly spaced prior on [−2, 2]
    ParticleFilter<float, 1, 4>  four      # Initialize(i ⇒ i), then Update(x ⇒ log(0.1·(x + 1))): weights (0.1, 0.2, 0.3, 0.4)
    Gaussian(z, σ) = [z, σ](x) ⇒ −½((z − x)/σ)²
# each case below is a TEST_F(TestParticleFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
initialize_sets_uniform_weights:
    Assert: every grid weight ≈ 1/41;  EffectiveSampleSize() ≈ 41
update_normalises_in_the_log_domain:
    Act:    grid.Update(Gaussian(0.5, 0.5))
    Assert: Mean() ≈ 0.498389;  EffectiveSampleSize() ≈ 17.690938;  MaximumWeight() ≈ 0.5;  Σ weights ≈ 1
    Act:    the same on a fresh grid with ℓ − 1000
    Assert: identical weights (no underflow)
sequential_updates_multiply_likelihoods:
    Act:    grid.Update(Gaussian(0.5, 0.5));  grid.Update(Gaussian(0.7, 0.5))
    Assert: Mean() ≈ 0.599970;  EffectiveSampleSize() ≈ 12.532656
systematic_resampling_is_deterministic:
    Assert: four.EffectiveSampleSize() ≈ 3.333333
            after four.Resample(0.5): particles (1, 2, 3, 3), weights all 0.25
            on a fresh `four`, Resample(0): particles (0, 1, 2, 3)
resample_if_needed_respects_the_threshold:
    Assert: four.ResampleIfNeeded(0.5, 0.5) == false (ESS 3.33 ≥ 2), particles unchanged
            four.ResampleIfNeeded(0.9, 0.5) == true  (ESS 3.33 < 3.6)
predict_moves_every_particle_and_keeps_weights:
    Act:    four.Predict([](x, i) { x[0] += 1; })
    Assert: Mean() ≈ 3.0 (2.0 + 1);  weights still (0.1, 0.2, 0.3, 0.4)
impossible_measurement_leaves_weights_unchanged:
    Assert: four.Update(x ⇒ −∞) == false;  weights still (0.1, 0.2, 0.3, 0.4)
```

## Reference vectors

- Grid prior on `[−2, 2]` (41 points), `z = 0.5, σ = 0.5`: posterior mean `0.498389` (the discretised
  Gaussian's mean is slightly pulled to 0 by the truncated prior), ESS `17.690938`.
- Second measurement `z = 0.7`: mean `0.599970` (≈ (0.5 + 0.7)/2), ESS `12.532656`.
- Systematic pointers for `u = 0.5`, `N = 4`: `0.125, 0.375, 0.625, 0.875` against the cumulative weights
  `0.1, 0.3, 0.6, 1.0` ⇒ indices `1, 2, 3, 3`.

## Edge cases

- `u` close to 1 (`0.9999`) still selects valid indices (the `j < N − 1` guard).
- A particle with weight 0 is never selected by `Resample`.
