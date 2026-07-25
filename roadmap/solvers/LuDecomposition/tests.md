# LU Decomposition — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestLuDecomposition : public ::testing::Test:
    LuDecomposition<float, 3> lu
    LuDecomposition<float, 4> lu4
# each case below is a TEST_F(TestLuDecomposition, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reconstructs_PA_equals_LU:
    Arrange: fixed non-symmetric 3×3 A
    Act:     Decompose(A)
    Assert:  P·A ≈ L·U

solves_linear_system:
    Arrange: A x = b with known x
    Assert:  Solve(b) ≈ x

determinant_matches_known:
    Arrange: 3×3 with hand-computed det
    Assert:  Determinant() ≈ reference (correct sign)

inverse_times_A_is_identity:
    Assert:  A · Inverse() ≈ I

pivoting_handles_zero_leading_pivot:
    Arrange: A with a[0][0] == 0 (forces a row swap)
    Assert:  factorization still reconstructs P·A

singular_matrix_returns_false:
    Arrange: rank-deficient A
    Assert:  Decompose returns false, singular flag set

multiple_rhs_reuse_factorization:
    Arrange: factor once, solve for two different b
    Assert:  both solutions correct (no re-factor)

matches_gaussian_elimination:
    Assert:  Solve(b) equals the solvers::GaussianElimination result (within tol)
```

## Reference vectors

- 3×3 requiring a pivot swap ⇒ documented `P`, `L`, `U`, and `det`.
- Cross-check `Solve` against `GaussianElimination` on the same system.

## Edge cases

- Singular / zero-pivot column ⇒ `false`, no divide-by-zero.
- Near-singular (large growth) ⇒ solution still bounded with pivoting.
- Identity and `1×1` trivial systems.
