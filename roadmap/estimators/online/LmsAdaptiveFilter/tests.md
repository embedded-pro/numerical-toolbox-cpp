# LMS / NLMS Adaptive Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestLmsAdaptiveFilter : public ::testing::Test:
    LmsAdaptiveFilter<float, 4> lms{ 0.1f }     # 4 taps, mu = 0.1
# each case below is a TEST_F(TestLmsAdaptiveFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
identifies_static_fir_system:
    Arrange: unknown plant w* = [0.5, -0.25, ...]; desired = w*ᵀx
    Act:     Update over many random inputs
    Assert:  weights converge to w*; error → 0

error_decreases_on_average:
    Arrange: stationary input
    Assert:  moving average of e² is non-increasing

zero_error_freezes_weights:
    Arrange: desired == current output (e = 0)
    Assert:  weights unchanged after Update

nlms_converges_independent_of_input_scale:
    Arrange: same system, input scaled ×10, normalized = true
    Assert:  convergence rate ≈ unscaled case

larger_mu_converges_faster:
    Arrange: two filters mu = 0.05 vs 0.2
    Assert:  higher mu reaches threshold in fewer samples

reset_clears_weights_and_history:
    Arrange: adapt, Reset()
    Assert:  Weights == 0; next output == 0

output_equals_fir_of_weights:
    Arrange: set known weights, feed an impulse
    Assert:  output sequence == weights (FIR identity)
```

## Reference vectors

- Two-tap plant `w* = [0.5, 0.5]`, white input ⇒ weights converge to `[0.5, 0.5]`.
- NLMS with `mu = 1` ⇒ one-step convergence for a single-tap noiseless system.

## Edge cases

- `mu` above the stability bound ⇒ divergence (assert detectable growth in a diagnostics test).
- Silent input (`‖x‖² ≈ 0`) with NLMS ⇒ `epsilon` prevents blow-up.
