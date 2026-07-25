# Matrix Exponential — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestMatrixExponential : public ::testing::Test:
    MatrixExponential<float, 2> expm2
    MatrixExponential<float, 3> expm3
# each case below is a TEST_F(TestMatrixExponential, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_matrix_gives_identity:
    Assert: Compute(0) ≈ I

diagonal_is_elementwise_exp:
    Arrange: A = diag(1, -2)
    Assert:  Compute(A) ≈ diag(e, e^-2)

scalar_1x1_matches_exp:
    Arrange: A = [a]
    Assert:  Compute(A) ≈ [exp(a)]

nilpotent_is_polynomial:
    Arrange: strictly upper-triangular N (N² = 0)
    Assert:  Compute(N) ≈ I + N   (series terminates)

rotation_generator:
    Arrange: A = [[0,-θ],[θ,0]]
    Assert:  Compute(A) ≈ [[cosθ,-sinθ],[sinθ,cosθ]]

known_2x2_reference:
    Arrange: Moler & Van Loan worked example
    Assert:  ≈ documented expm

large_norm_uses_scaling:
    Arrange: A with ‖A‖ ≫ 1
    Assert:  matches reference (scaling/squaring engaged), no overflow

exp_zero_dt_is_identity:
    Assert: Compute(A, 0) ≈ I
```

## Reference vectors

- `A = [[0,1],[0,0]]` ⇒ `expm = [[1,1],[0,1]]`.
- `A = [[0,-1],[1,0]]` (θ = 1) ⇒ `[[cos1,-sin1],[sin1,cos1]]`.
- Moler & Van Loan (2003) canonical 2×2 example ⇒ tabulated result.

## Edge cases

- Nearly-defective matrix (close eigenvalues) ⇒ still accurate via scaling/squaring.
- Large negative eigenvalues (stiff) ⇒ result stays bounded, no overflow.
- `N = 1` degenerate matrix reduces to a scalar `exp`.
