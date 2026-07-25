# Symmetric Eigenvalue Solver (Jacobi) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestJacobiEigenSolver : public ::testing::Test:
    JacobiEigenSolver<float, 3> solver
    JacobiEigenSolver<float, 2> solver2
# each case below is a TEST_F(TestJacobiEigenSolver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
diagonal_matrix_returns_diagonal:
    Arrange: A = diag(3, 1, 2)
    Assert:  eigenvalues ≈ {3, 2, 1} (sorted), eigenvectors ≈ axes

reconstructs_A:
    Assert:  V · diag(λ) · Vᵀ ≈ A

eigenvectors_orthonormal:
    Assert:  Vᵀ·V ≈ I

known_symmetric_2x2:
    Arrange: [[2,1],[1,2]]
    Assert:  λ ≈ {3, 1}, vectors ≈ (1,±1)/√2

eigenpairs_satisfy_definition:
    Assert:  A·v_i ≈ λ_i·v_i for every pair

off_diagonal_norm_decreases:
    Assert:  OffDiagonalNorm shrinks monotonically across sweeps

repeated_eigenvalues_handled:
    Arrange: A with a double eigenvalue
    Assert:  reconstructs A, eigenvectors still orthonormal

identity_has_unit_eigenvalues:
    Assert:  λ ≈ {1,1,1}, converges in zero sweeps

converges_within_max_sweeps:
    Assert:  Decompose returns true (converged flag)
```

## Reference vectors

- `[[2,1],[1,2]]` ⇒ `λ = {3,1}`, vectors `(1,1)/√2` and `(1,−1)/√2`.
- Diagonal input ⇒ eigenvalues equal the diagonal, eigenvectors the identity.

## Edge cases

- Already-diagonal input ⇒ no rotations performed.
- Repeated / clustered eigenvalues ⇒ eigenvectors non-unique but orthonormal.
- Non-symmetric input ⇒ assertion (documented precondition).
- `N == 1` trivial case.
