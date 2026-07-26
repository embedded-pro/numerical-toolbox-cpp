# Discrete Wavelet Transform — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestDiscreteWaveletTransform : public ::testing::Test:
    static constexpr size_t N = 16
    WaveletFilters<float, 2> haar = MakeHaar()
    WaveletFilters<float, 4> db2  = MakeDaubechies2()
    DiscreteWaveletTransform<float, N, 3> dwtHaar{ haar }
# each case below is a TEST_F(TestDiscreteWaveletTransform, <name>)
```

## Test cases (Arrange / Act / Assert)

```
haar_of_constant_puts_energy_in_approx:
    Arrange: x = constant 1.0
    Act:     Forward(x, coeffs)
    Assert:  detail coefficients ≈ 0, approximation holds the energy

haar_detail_captures_step_edge:
    Arrange: x = step (0…0, 1…1)
    Assert:  one detail coefficient at the edge is large, others ≈ 0

perfect_reconstruction_haar:
    Arrange: random x
    Act:     Inverse(Forward(x))
    Assert:  reconstructed ≈ x (±1e-5)

perfect_reconstruction_daubechies:
    Arrange: db2 filters, random x
    Assert:  round-trip ≈ x

energy_is_preserved_orthogonal:
    Assert:  Σ x^2 ≈ Σ coeffs^2 (Parseval for an orthogonal wavelet)

single_level_matches_manual_qmf:
    Arrange: Levels = 1, small x
    Assert:  cA, cD match a hand-computed convolve + downsample

multilevel_offsets_are_consistent:
    Assert:  LevelOffset partitions coeffs without overlap; sizes N/2, N/4, N/8

linearity_holds:
    Assert:  Forward(a·x1 + b·x2) ≈ a·Forward(x1) + b·Forward(x2)
```

## Reference vectors

- Haar, constant input ⇒ all detail bands 0, approximation = scaled constant.
- Haar of `[a, b]` (one stage) ⇒ `cA = (a+b)/√2`, `cD = (a−b)/√2`.
- Orthogonal wavelet ⇒ Parseval energy equality.

## Edge cases

- `N` not divisible by `2^Levels` (precondition / `static_assert`).
- Boundary-extension consistency (periodic indexing) at block edges.
- All-zero input ⇒ all-zero coefficients and reconstruction.
- Maximum decomposition depth (`Levels = log2 N`).
