# Complementary Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestComplementaryFilter : public ::testing::Test:
    ComplementaryFilter<float> filter{ 0.98f, 0.01f }   # alpha = 0.98, Ts = 10 ms
# each case below is a TEST_F(TestComplementaryFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
static_measurement_converges:
    Arrange: rate = 0, measuredAngle = 0.2 held for many samples
    Act:     iterate Update(0, 0.2)
    Assert:  angle -> 0.2  (low-pass path wins at DC)

pure_rotation_follows_gyro:
    Arrange: constant rate w, measuredAngle = 0 (unusable slow sensor)
    Assert:  short-term angle ≈ integral(w) = w * Ts * n

rejects_slow_sensor_noise:
    Arrange: rate = 0, measuredAngle = 0 + high-frequency noise
    Assert:  variance(angle) < variance(measuredAngle)

rejects_gyro_bias_drift:
    Arrange: rate = small constant bias, measuredAngle = 0 (true)
    Assert:  angle stays bounded (does not integrate to infinity)

alpha_from_tau_matches_crossover:
    Arrange: a = AlphaFromTau(tau, Ts)
    Assert:  a == tau/(tau+Ts) within tolerance

heading_blend_wraps_shortest_arc:
    Arrange: wrapAngle = true, predicted ≈ +pi-eps, measuredAngle ≈ -pi+eps
    Assert:  fused angle stays near ±pi, no 2*pi jump

reset_sets_state:
    Arrange: run samples, Reset(0.1)
    Assert:  internal angle == 0.1

alpha_zero_is_pure_measurement / alpha_one_is_pure_integration:
    Assert:  alpha=0 ⇒ output == measuredAngle; alpha=1 ⇒ output == running gyro integral
```

## Reference vectors

- DC step on the slow sensor ⇒ first-order low-pass rise toward the measured angle.
- Constant gyro rate with dead slow sensor ⇒ output equals the discrete integral `w*Ts*n`.

## Edge cases

- Heading wrap across the ±pi seam (both directions).
- `alpha` at bounds (0 ⇒ measurement only, 1 ⇒ integration only, drift unbounded).
- Long run under constant bias ⇒ bounded steady-state error `≈ bias*Ts*alpha/(1-alpha)`.
