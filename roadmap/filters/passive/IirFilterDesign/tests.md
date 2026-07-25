# IIR Filter Design — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestIirFilterDesign : public ::testing::Test:
    IirFilterDesign<float, 8> designer{}   # up to 8th order
# each case below is a TEST_F(TestIirFilterDesign, <name>)
```

## Test cases (Arrange / Act / Assert)

```
butterworth_lp_section_count:
    Arrange: order = 4 low-pass
    Act:     n = Design(Butterworth, LowPass, 4, 100, 1000)
    Assert:  n == 2 sections (order/2)

butterworth_lp_dc_gain_unity:
    Arrange: design 2nd-order Butterworth LP, build a BiquadCascade
    Assert:  DC magnitude ≈ 1  (0 dB passband)

cutoff_is_minus_3db:
    Arrange: Butterworth LP fc = 100, fs = 1000
    Assert:  |H(fc)| ≈ 1/√2  (−3 dB point, from pre-warp)

prewarp_places_cutoff_exactly:
    Arrange: sweep fc up toward Nyquist
    Assert:  measured −3 dB frequency matches the requested fc within tol

chebyshev_has_passband_ripple:
    Arrange: Chebyshev-I, rippleDb = 1, order = 4
    Assert:  passband oscillates within 1 dB, steeper roll-off than Butterworth

highpass_blocks_dc:
    Arrange: Design(Butterworth, HighPass, 2, 100, 1000)
    Assert:  DC magnitude ≈ 0

design_is_stable:
    Arrange: several orders / cutoffs
    Assert:  every emitted pole lies strictly inside the unit circle
```

## Reference vectors

- 2nd-order Butterworth LP, `fc/fs = 0.1` ⇒ compare `{b,a}` against a reference design tool (±tol).
- `|H(fc)| = 0.7071` for any Butterworth order at the design cutoff.

## Edge cases

- `order = 1` (single real pole → one first-order section padded as a biquad).
- `fc` approaching `0` or Nyquist — pre-warp stays finite; assert stable coefficients.
- Odd order ⇒ one real-pole section plus conjugate-pair sections.
