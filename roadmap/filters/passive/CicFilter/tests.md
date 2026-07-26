# CIC Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCicDecimator : public ::testing::Test:
    CicDecimator<float, 2, 4, 1> cic{}     # Stages=2, R=4, M=1
# each case below is a TEST_F(TestCicDecimator, <name>)
```

## Test cases (Arrange / Act / Assert)

```
emits_one_per_R:
    Arrange: R = 4, feed 8 samples
    Act:     count non-empty outputs
    Assert:  exactly 2 outputs produced (decimation by 4)

dc_gain_normalized:
    Arrange: constant input c, Stages=2, R=4, M=1
    Assert:  normalized output ≈ c  (gain (R*M)^Stages divided out)

impulse_response_is_triangular:
    Arrange: single impulse, Stages=2, M=1
    Assert:  decimated impulse response matches the sinc^2 / triangular shape

silence_gives_zero:
    Arrange: all-zero input
    Assert:  every emitted output == 0

reset_clears_all_state:
    Arrange: run samples, Reset()
    Assert:  integrators, combs, and phase all zero
```

## Reference vectors

- Stages=1, R=4, M=1, constant `1` ⇒ each output = `4` before normalization (gain `R·M`).
- Stages=2, R=2, M=1, impulse ⇒ decimated response `[1, 2, 1]` (triangular) pre-normalization.

## Edge cases

- `R = 1` ⇒ no decimation (integrator-comb pair only).
