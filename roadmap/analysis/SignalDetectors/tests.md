# Signal Detectors — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSignalDetectors : public ::testing::Test:
    PeakHold<float>            peak{ 1.0f }    # no decay by default
    ZeroCrossingCounter<float> zc{ 0.0f }
    RmsEnvelope<float>         rms{ 0.25f }
# each case below is a TEST_F(TestSignalDetectors, <name>)
```

## Test cases (Arrange / Act / Assert)

```
peak_hold_latches_maximum:
    Arrange: feed [0.2, 0.5, 0.3, 0.1]
    Act:     collect outputs
    Assert:  output stays at 0.5 after the peak (decay = 1)

peak_hold_decays_when_leaky:
    Arrange: decay = 0.5, feed [0.8, 0, 0, 0]
    Assert:  peak ≈ [0.8, 0.4, 0.2, 0.1]

peak_hold_tracks_magnitude_not_sign:
    Arrange: feed [-0.9, 0.1]
    Assert:  first output == 0.9

zero_crossing_counts_sign_changes:
    Arrange: feed [1, -1, 1, -1]
    Assert:  Count() == 3

zero_crossing_hysteresis_rejects_noise:
    Arrange: hysteresis = 0.1, feed dither [0.02, -0.02, 0.02]
    Assert:  Count() == 0

rms_envelope_converges_to_true_rms:
    Arrange: alpha = 0.1, feed a sine of RMS 0.5 for many samples
    Assert:  steady-state output ≈ 0.5 (±tol)

rms_dc_input_returns_magnitude:
    Arrange: feed constant 0.4
    Assert:  output → 0.4

reset_clears_all_state:
    Arrange: run samples, Reset()
    Assert:  peak == 0, Count() == 0, rms == 0
```

## Reference vectors

- Peak-hold, `decay = 0.5`, impulse `0.8` ⇒ `0.8·0.5^n`.
- Full-scale sine of amplitude `A` ⇒ RMS envelope → `A/√2`.
- Square wave of period `P` samples ⇒ 2 crossings per period.

## Edge cases

- All-zero input: no crossings, RMS stays 0, peak stays 0.
- Constant-sign ramp: `ZeroCrossingCounter` reports 0.
- Single-sample and alternating ±full-scale sequences.
