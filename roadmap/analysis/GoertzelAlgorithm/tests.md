# Goertzel Algorithm — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestGoertzel : public ::testing::Test:
    static constexpr size_t N = 16
    # each test constructs Goertzel<float> with a chosen bin k
# each case below is a TEST_F(TestGoertzel, <name>)
```

## Test cases (Arrange / Act / Assert)

```
detects_matching_tone:
    Arrange: k = 2, feed N samples of cos(2π·2·n/N)
    Act:     push all, read Magnitude()
    Assert:  Magnitude() large (≈ N/2), Ready() true

rejects_off_bin_tone:
    Arrange: k = 2, feed a tone at bin 5
    Assert:  Magnitude() ≈ 0 (small leakage)

magnitude_matches_dft_bin:
    Arrange: arbitrary length-N signal
    Assert:  Result() ≈ reference DFT X[k] (±tol)

coefficient_formula_correct:
    Assert:  Coefficient(k, N) ≈ 2·cos(2π·k/N)

dc_bin_sums_input:
    Arrange: k = 0, constant input c
    Assert:  Result().Real ≈ N·c

magnitude_squared_matches_result:
    Assert:  Magnitude()^2 ≈ Real^2 + Imag^2 of Result()

reset_restarts_block:
    Arrange: push samples, Reset(), push a fresh block
    Assert:  result depends only on the second block

not_ready_before_n_samples:
    Arrange: push N-1 samples
    Assert:  Ready() == false
```

## Reference vectors

- On-bin cosine of amplitude 1 ⇒ `|X[k]| ≈ N/2`.
- DC (`k = 0`), constant `c` ⇒ `X[0] = N·c`.
- Compare against a direct `Σ x[n]·e^{−j2πkn/N}` reference for a random block.

## Edge cases

- `k = 0` and `k = N/2` (real-only bins).
- Feeding more than `N` samples without `Reset` (documented drift).
- Single-sample block (`N = 1`).
