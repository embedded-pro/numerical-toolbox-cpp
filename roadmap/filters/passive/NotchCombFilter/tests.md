# Notch / Comb Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestNotchComb : public ::testing::Test:
    NotchFilter<float>    notch{ 50.0f, 1000.0f, 10.0f }   # 50 Hz, fs=1 kHz, Q=10
    CombFilter<float, 20> comb{ 0.9f }                     # D=20 (fs/f0 = 1000/50)
# each case below is a TEST_F(TestNotchComb, <name>)
```

## Test cases (Arrange / Act / Assert)

```
notch_rejects_target_frequency:
    Arrange: feed a 50 Hz sine (fs = 1 kHz)
    Assert:  steady-state output amplitude ≈ 0

notch_passes_off_frequency:
    Arrange: feed a 200 Hz sine
    Assert:  output amplitude ≈ input amplitude (≈ 0 dB)

notch_dc_gain_unity:
    Arrange: constant input
    Assert:  output ≈ input (DC untouched)

comb_rejects_harmonics:
    Arrange: feedforward comb D=20, feed fundamental + 2nd/3rd harmonic of fs/D
    Assert:  all harmonics of fs/D attenuated

feedback_comb_is_stable:
    Arrange: gain = 0.95, feed impulse
    Assert:  output decays (bounded), no growth

reset_clears_state:
    Arrange: run samples, Reset()
    Assert:  biquad state and delay line all zero
```

## Reference vectors

- Notch `f0/fs = 0.05, Q = 10` ⇒ magnitude at `0.05·fs` ≈ `0`, at DC and Nyquist ≈ `1`.
- FIR comb `gain = 1, D = 4` ⇒ zeros at `k·fs/4`; impulse response `[1,0,0,0,−1]`.

## Edge cases

- Feedback `gain` approaching 1 — assert bounded output (stability margin).
- `D` not integer-dividing `fs/f0` — documents residual-hum detuning.
