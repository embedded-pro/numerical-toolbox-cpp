# Biquad / SOS Cascade — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestBiquadCascade : public ::testing::Test:
    BiquadCascade<float, 2> filter{ /* two low-pass sections */ }
# each case below is a TEST_F(TestBiquadCascade, <name>)
```

## Test cases (Arrange / Act / Assert)

```
dc_gain_of_lowpass_is_unity:
    Arrange: LP section fc = 0.1*fs, feed constant 1
    Assert:  steady-state output ≈ 1  (0 dB at DC)

lowpass_attenuates_high_freq:
    Arrange: feed a sine near Nyquist through the LP cascade
    Assert:  output amplitude << input amplitude

bypass_coeffs_are_passthrough:
    Arrange: b0=1, b1=b2=a1=a2=0
    Assert:  Filter(x) == x

known_impulse_response:
    Arrange: single section with fixed coeffs
    Act:     collect first few impulse-response samples
    Assert:  matches hand-computed h[0..4]

cascade_equals_serial_sections:
    Arrange: 2-section cascade vs two standalone biquads chained
    Assert:  identical outputs

reset_clears_state:
    Arrange: run samples, Reset()
    Assert:  z1,z2 == 0 in every section; next impulse gives b0
```

## Reference vectors

- Bypass `{b0=1,b1=0,b2=0,a1=0,a2=0}` ⇒ impulse response `[1,0,0,...]`.
- RBJ LP `fc/fs = 0.25, Q = 0.707` ⇒ compare `{b,a}` against cookbook constants (±tol).

## Edge cases

- Coefficients near instability (poles close to the unit circle) — bounded output for bounded input.
- Very small (`float` denormal) states do not stall the pipeline.
