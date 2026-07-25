# Hilbert Transform / Analytic Signal — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestHilbertTransform : public ::testing::Test:
    static constexpr size_t N = 64
    FastFourierTransformRadix2Impl<float, N> engine
    AnalyticSignalFft<float, N> hilbert{ engine }
    HilbertFir<float, 31> fir
# each case below is a TEST_F(TestHilbertTransform, <name>)
```

## Test cases (Arrange / Act / Assert)

```
cosine_maps_to_sine_imag:
    Arrange: x = cos(2π·f·n)
    Act:     a = Analytic(x)
    Assert:  Real(a) ≈ cos, Imag(a) ≈ sin  (H{cos} = sin), interior samples

analytic_real_part_equals_input:
    Assert:  Real(Analytic(x)) ≈ x (edge-trimmed)

envelope_of_am_signal_is_modulator:
    Arrange: x = (1 + 0.5·cos(ω_m n))·cos(ω_c n), ω_c ≫ ω_m
    Assert:  InstantaneousAmplitude tracks (1 + 0.5·cos(ω_m n)) (±tol)

instantaneous_frequency_of_tone_is_constant:
    Arrange: pure tone at f0
    Assert:  InstantaneousFrequency ≈ f0 across interior samples

instantaneous_frequency_tracks_chirp:
    Arrange: linear chirp f(t) = f0 + r·t
    Assert:  estimated frequency increases linearly (±tol)

negative_frequencies_are_zeroed:
    Assert:  spectrum of the analytic signal has ~0 energy for k in (N/2, N)

fir_matches_fft_in_passband:
    Arrange: mid-band tone
    Assert:  FIR envelope ≈ FFT envelope (±tol) after group-delay alignment

dc_input_has_zero_hilbert:
    Arrange: constant input
    Assert:  Imag(Analytic) ≈ 0
```

## Reference vectors

- `H{cos(ωn)} = sin(ωn)`, `H{sin(ωn)} = −cos(ωn)`.
- AM signal envelope equals the modulating waveform.
- Analytic-signal spectrum is one-sided (zero for negative frequencies).

## Edge cases

- Edge/transient samples of the FFT block (window or trim before asserting).
- Phase unwrapping across the `±π` boundary.
- DC and Nyquist bins (imaginary part must stay zero).
- FIR group-delay alignment when comparing to the raw input.
