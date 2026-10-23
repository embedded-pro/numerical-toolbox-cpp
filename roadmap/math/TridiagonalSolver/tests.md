# Tridiagonal Solver — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestTridiagonalSolver : public ::testing::Test:
    TridiagonalSolver<float, 8> solver
    # second-difference matrix tridiag(−1, 2, −1), n = 4
    Band sub{ 0, −1, −1, −1 }, diag{ 2, 2, 2, 2 }, sup{ −1, −1, −1, 0 }
# each case below is a TEST_F(TestTridiagonalSolver, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
solves_second_difference_system:
    Arrange: Factor(sub, diag, sup, 4);  b = (1, 0, 0, 1)
    Assert:  Solve(b) ≈ (1, 1, 1, 1)
one_factorization_serves_several_right_hand_sides:
    Arrange: Factor once;  b₁ = (1, 0, 0, 1), b₂ = (1, 2, 3, 4)
    Assert:  Solve(b₁) ≈ (1, 1, 1, 1)  and  Solve(b₂) ≈ (4, 7, 8, 6)
single_row_system_divides_by_the_diagonal:
    Assert: Factor({0}, {4}, {0}, 1) and Solve({2}) ≈ (0.5)
vanishing_pivot_is_reported:
    Arrange: diag = (1, 1, …), sub = sup = −1 ⇒ second pivot 1 − 1·1 = 0
    Assert:  Factor(...) == false
size_outside_range_is_rejected:
    Assert: Factor(..., 0) == false  and  Factor(..., 9) == false
```

## Reference vectors

- `tridiag(−1, 2, −1)·(1,1,1,1) = (1,0,0,1)`; `tridiag(−1, 2, −1)·(4,7,8,6) = (1,2,3,4)`.

## Edge cases

- Strictly diagonally dominant random systems (`|b_k| > |a_k| + |c_k|`) ⇒ residual `‖A·x − b‖ ≤ 1e-5·‖b‖`.
- Entries of the returned band at index `≥ n` are zero.
