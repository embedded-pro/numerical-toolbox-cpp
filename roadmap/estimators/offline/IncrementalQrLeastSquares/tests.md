# Incremental QR Least Squares — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestIncrementalQrLeastSquares : public ::testing::Test:
    IncrementalQrLeastSquares<float, 2> line            # rows (1, x)
    IncrementalQrLeastSquares<float, 3> redundant       # rows (1, x, x) — rank 2
    std::array<float, 4> x{ 0, 1, 2, 3 }
# each case below is a TEST_F(TestIncrementalQrLeastSquares, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
exact_data_recovers_parameters:
    Arrange: AddRow((1, xᵢ), 2 + 3xᵢ) for every i
    Assert:  Solve(1e-5) → parameters ≈ (2, 3), rank == 2, residualRms ≈ 0
noisy_data_matches_batch_least_squares:
    Arrange: τ = (2.1, 4.9, 8.1, 10.9)
    Assert:  parameters ≈ (2.06, 2.96), residualRms ≈ 0.089443
rank_deficient_regressor_returns_minimum_norm_solution:
    Arrange: redundant.AddRow((1, xᵢ, xᵢ), 2 + 3xᵢ)
    Assert:  rank == 2;  parameters ≈ (2, 1.5, 1.5);  rowSpace row 2 ∥ (0, 1, −1)
row_order_does_not_change_the_result:
    Assert: feeding the noisy rows reversed gives the same parameters and residualRms (tol 1e-5)
block_rows_equal_single_rows:
    Assert: AddRows(4×2 block, τ) gives the same result as four AddRow calls
no_rows_returns_nullopt:
    Assert: Solve(1e-5) == std::nullopt, also after Reset()
```

## Reference vectors

- `x = (0, 1, 2, 3)`, `τ = (2.1, 4.9, 8.1, 10.9)`: least squares `(2.06, 2.96)`, residual RMS `0.089443`.
- Rows `(1, x, x)`, `τ = 2 + 3x`: `σ = (5.540482, 1.141517, 0)`, minimum-norm `(2, 1.5, 1.5)`, null
  direction `(0, 1, −1)/√2` (python3/numpy).

## Edge cases

- One row only ⇒ rank 1, finite result.
- Large row count (10⁵ rows of the line model) ⇒ same parameters; no growth in memory.
- Rows that are exact multiples of earlier ones ⇒ rank unchanged.
