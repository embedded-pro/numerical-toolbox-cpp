# Real-Input FFT (RFFT) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestRealFft : public ::testing::Test:
    static constexpr size_t N = 16
    FastFourierTransformRadix2Impl<float, N/2> engine   # real complex FFT, injected
    RealFft<float, N> rfft{ engine }
    BoundedVector<float>::WithMaxSize<N> input
# each case below is a TEST_F(TestRealFft, <name>)
```

## Test cases (Arrange / Act / Assert)

```
dc_input_has_only_dc_bin:
    Arrange: input = constant 1.0 (N samples)
    Act:     spectrum = rfft.Forward(input)
    Assert:  spectrum[0].Real ≈ N, all other bins ≈ 0

single_real_sinusoid_hits_one_bin:
    Arrange: input = cos(2π·3·n/N)
    Assert:  |spectrum[3]| peaks, neighbours ≈ 0

matches_reference_complex_fft:
    Arrange: random real input
    Act:     compare rfft.Forward against a full complex FFT of the same signal
    Assert:  bins 0..N/2 equal (±tol)

nyquist_and_dc_are_real:
    Assert:  spectrum[0].Imag == 0 and spectrum[N/2].Imag == 0

inverse_is_left_inverse:
    Arrange: y = rfft.Inverse(rfft.Forward(input))
    Assert:  y ≈ input (round-trip, ±tol)

linearity_holds:
    Assert:  Forward(a·x1 + b·x2) ≈ a·Forward(x1) + b·Forward(x2)

impulse_has_flat_magnitude:
    Arrange: input = unit impulse
    Assert:  |spectrum[k]| ≈ 1 for all k

rejects_non_power_of_two:
    Arrange: N not a power of two
    Assert:  static_assert / precondition fails at compile time
```

## Reference vectors

- Constant `1` ⇒ `X[0] = N`, rest 0.
- Unit impulse ⇒ flat magnitude spectrum.
- On-bin cosine at bin `k` ⇒ energy only in bin `k` (and its Hermitian mirror).

## Edge cases

- Smallest size (`N = 4`) and a larger power of two.
- All-zero input ⇒ all-zero spectrum, stable inverse.
- Round-trip accumulation error stays within tolerance.
- Nyquist bin present only when `N` is even (always here).
