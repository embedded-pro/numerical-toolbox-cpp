# QR Decomposition — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestQrDecomposition : public ::testing::Test:
    QrDecomposition<float, 4, 3> qr        # tall, overdetermined
    QrDecomposition<float, 3, 3> qrSquare
# each case below is a TEST_F(TestQrDecomposition, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reconstructs_A:
    Arrange: fixed 4×3 A
    Act:     Decompose(A)
    Assert:  Q·R ≈ A  (within 1e-5)

Q_columns_are_orthonormal:
    Assert:  Qᵀ·Q ≈ I  (Cols×Cols)

R_is_upper_triangular:
    Assert:  entries below the diagonal of R are ≈ 0

solves_overdetermined_least_squares:
    Arrange: line-fit system with more rows than unknowns
    Assert:  solution matches the normal-equation reference (within tol)

matches_known_3x3:
    Arrange: golden 3×3 from Golub & Van Loan
    Assert:  R (up to column sign) equals the documented factor

apply_qtranspose_equals_explicit:
    Assert:  ApplyQtranspose(b) == Q()ᵀ · b

rank_deficient_returns_false:
    Arrange: A with a dependent column
    Assert:  Decompose returns false (near-zero R pivot)

givens_update_adds_row:
    Arrange: factor A, then stream one extra row via Givens
    Assert:  updated R matches a full re-factorization of the augmented A

identity_factors_to_identity:
    Assert:  Q ≈ I, R ≈ I
```

## Reference vectors

- Golden 3×3 with documented `Q`,`R` (sign convention fixed by `−sign(x₀)‖x‖`).
- Overdetermined fit ⇒ least-squares solution equal to the `(AᵀA)⁻¹Aᵀb` reference.

## Edge cases

- Rank-deficient / dependent columns ⇒ zero pivot flagged.
- Square vs tall shapes; `Cols == 1` (a single reflector).
- Sign ambiguity of columns — assertions compare up to per-column sign.
