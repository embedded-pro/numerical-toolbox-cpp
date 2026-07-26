# Convolution & Correlation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestConvolutionCorrelation : public ::testing::Test:
    Vector<float, 16> x, h, y, r     # WithMaxSize buffers, filled per test
# each case below is a TEST_F(TestConvolutionCorrelation, <name>)
```

## Test cases (Arrange / Act / Assert)

```
linear_convolution_matches_hand_calc:
    Arrange: x = [1, 2, 3], h = [0, 1, 0.5]
    Act:     LinearConvolution(x, h, y)
    Assert:  y ≈ [0, 1, 2.5, 4, 1.5]

convolution_with_unit_impulse_is_identity:
    Arrange: h = [1, 0, 0]
    Assert:  first |x| samples of y == x

convolution_is_commutative:
    Assert:  conv(x, h) ≈ conv(h, x)

circular_convolution_wraps:
    Arrange: length-4 x and h
    Assert:  matches a DFT-multiply reference

autocorrelation_peaks_at_zero_lag:
    Arrange: r = AutoCorrelation(x)
    Assert:  argmax(r) is the zero-lag index, r symmetric

cross_correlation_finds_known_delay:
    Arrange: y = shift(x, +3)
    Assert:  ArgMaxLag(CrossCorrelation(x, y)) == 3

fast_convolution_matches_direct:
    Arrange: StrictMock<Fft> (or real radix-2 FFT), power-of-two length
    Assert:  FastConvolution ≈ LinearConvolution (±tol)

single_tap_scales_signal:
    Arrange: h = [k]
    Assert:  y == k · x
```

## Reference vectors

- `[1,2,3] * [1,1]` (linear) ⇒ `[1, 3, 5, 3]`.
- Auto-correlation of `[1,1,1,1]` ⇒ triangular `[1,2,3,4,3,2,1]` (unnormalized).
- A delayed copy at lag `d` ⇒ cross-correlation peak at `d`.

## Edge cases

- Different-length operands (M ≠ K).
- Zero input vector ⇒ zero output.
- Circular vs linear mismatch when not zero-padded (documented behaviour).
