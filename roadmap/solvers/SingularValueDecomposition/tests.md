# Singular Value Decomposition — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSingularValueDecomposition : public ::testing::Test:
    SingularValueDecomposition<float, 4, 3> svd        # tall
    SingularValueDecomposition<float, 3, 3> svdSquare
# each case below is a TEST_F(TestSingularValueDecomposition, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reconstructs_A:
    Arrange: fixed 4×3 A
    Act:     Decompose(A)
    Assert:  U · diag(σ) · Vᵀ ≈ A

singular_values_descending_and_nonnegative:
    Assert:  σ[0] >= σ[1] >= ... >= 0

U_and_V_orthonormal:
    Assert:  Uᵀ·U ≈ I and Vᵀ·V ≈ I

diagonal_matrix_gives_absolute_diagonal:
    Arrange: A = diag(3, −1, 2)
    Assert:  σ ≈ {3, 2, 1}

sigma_squared_matches_eig_AtA:
    Assert:  σ² ≈ eigenvalues(AᵀA)  (cross-check JacobiEigenSolver)

pseudo_inverse_solves_least_squares:
    Arrange: overdetermined system
    Assert:  A⁺·b matches the QR least-squares solution

rank_detection_thresholds_small_sigma:
    Arrange: rank-2 matrix living in a 3-column space
    Assert:  Rank(tol) == 2

condition_number_matches_known:
    Arrange: A with prescribed σ_max, σ_min
    Assert:  ConditionNumber() ≈ σ_max/σ_min

known_2x2_svd:
    Assert:  σ equals the documented Golub reference
```

## Reference vectors

- `diag(3, −1, 2)` ⇒ `σ = {3, 2, 1}` (ordered, absolute value).
- `σ(A)² = eigenvalues(AᵀA)` — golden cross-check against `JacobiEigenSolver`.

## Edge cases

- Rank-deficient / zero singular value ⇒ pseudo-inverse truncates without divide-by-zero.
- Wide matrix (`Rows < Cols`) ⇒ decompose `Aᵀ`; assert consistent factors.
- All-zero matrix ⇒ all `σ = 0`, `Rank == 0`.
- Repeated singular values ⇒ vectors non-unique but the subspace is correct.
