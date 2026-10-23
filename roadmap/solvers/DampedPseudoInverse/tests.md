# Damped Pseudo-Inverse and Null-Space Projector — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestDampedPseudoInverse : public ::testing::Test:
    math::Matrix<float, 2, 3> a{ { 1, 0, 1 }, { 0, 1, 1 } }      # σ = (√3, 1), null space ∥ (1, 1, −1)
    math::Vector<float, 2>    b{ 1, 2 }
    DampedPseudoInverse<float, 2, 3> exact{ 0.0f }
    DampedPseudoInverse<float, 2, 3> damped{ 0.1f }
    void SetUp(): ASSERT_TRUE(exact.Factor(a)) and ASSERT_TRUE(damped.Factor(a))
# each case below is a TEST_F(TestDampedPseudoInverse, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
undamped_solve_is_minimum_norm_exact_solution:
    Assert: exact.Solve(b) ≈ (0, 1, 1)  and  a·exact.Solve(b) ≈ b
damped_solve_error_is_bounded:
    Assert: damped.Solve(b) ≈ (0.003289, 0.993388, 0.996678);
            ‖a·damped.Solve(b) − b‖ ≈ 0.009934 ≤ λ²/σ_min²·‖b‖ = 0.022361
projection_is_invisible_to_the_matrix_despite_damping:
    Assert: damped.Project((1, 0, 0)) ≈ (1/3, 1/3, −1/3)  and  a·damped.Project((1, 0, 0)) ≈ 0
projector_is_an_orthogonal_projector:
    Arrange: P = damped.NullSpaceProjector()
    Assert:  P·P ≈ P,  Pᵀ ≈ P,  trace P ≈ 1 (= Cols − rank)
undamped_inverse_is_a_right_inverse:
    Assert: a·exact.Inverse() ≈ I₂
solve_has_no_null_space_component:
    Assert: damped.Project(damped.Solve(b)) ≈ 0
rank_loss_grows_the_null_space:
    Arrange: rank-1 matrix { {1, 1, 0}, {2, 2, 0} } (σ = (√10, 0))
    Assert:  Rank() == 1;  trace NullSpaceProjector() ≈ 2;  exact.Solve((1, 2)) ≈ (0.5, 0.5, 0)
```

## Reference vectors

- `a = [[1,0,1],[0,1,1]]`: `σ = (1.732051, 1)`, `A⁺b = (0, 1, 1)`, `P = (1/3)·[[1,1,−1],[1,1,−1],[−1,−1,1]]`.
- `λ = 0.1`: `A⁺_λb = (0.003289, 0.993388, 0.996678)`, residual `0.009934` (python3/numpy).

## Edge cases

- Tall matrix (`Rows > Cols`, full column rank) ⇒ `NullSpaceProjector() ≈ 0`, `Solve` is least squares.
- `λ = 0` with an exactly singular direction ⇒ zero gain on it (no division by zero).
- Zero matrix ⇒ `Rank() == 0`, `Solve` returns zero, `Project` is the identity.
