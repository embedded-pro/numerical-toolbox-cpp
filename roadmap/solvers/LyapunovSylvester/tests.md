# Lyapunov / Sylvester Solvers — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSylvesterSolver : public ::testing::Test:
    SylvesterSolver<float, 2, 2> sylv
    SylvesterSolver<float, 3, 3> lyap3
# each case below is a TEST_F(TestSylvesterSolver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
sylvester_reconstructs_C:
    Arrange: fixed A, B, C
    Act:     SolveSylvester(A, B, C)
    Assert:  A·X + X·B ≈ C

continuous_lyapunov_is_spd_for_stable_A:
    Arrange: Hurwitz A, Q = I
    Assert:  X symmetric, positive-definite, and A·X + X·Aᵀ ≈ −Q

discrete_lyapunov_matches_known:
    Arrange: Schur-stable A, Q = I
    Assert:  A·X·Aᵀ − X ≈ −Q

scalar_case_closed_form:
    Arrange: 1×1  a·x + x·a = c
    Assert:  x ≈ c / (2a)

symmetric_solution_for_symmetric_Q:
    Assert:  X ≈ Xᵀ

gramian_matches_truncated_sum:
    Arrange: controllability Gramian via discrete Lyapunov
    Assert:  X ≈ Σ_{k=0..K} Aᵏ B Bᵀ (Aᵀ)ᵏ  (K large)

matches_hand_computed_2x2:
    Assert:  X equals the documented Bartels-Stewart reference

unsolvable_pair_flagged:
    Arrange: A, B with λ_i + λ_j = 0
    Assert:  Solve returns false
```

## Reference vectors

- Scalar `a x + x a = c ⇒ x = c/(2a)`.
- 2×2 stable `A`, `Q = I` ⇒ hand-solved symmetric positive-definite `X`.

## Edge cases

- Eigenvalue pair summing to zero (continuous) or product = 1 (discrete) ⇒ no unique solution.
- Near-degenerate (tiny denominator) ⇒ ill-conditioned; flag a large solution norm.
- `N == 1` scalar; diagonal `A` closed-form cross-check.
