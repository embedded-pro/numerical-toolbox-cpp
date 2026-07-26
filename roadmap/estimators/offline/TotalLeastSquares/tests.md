# Total Least Squares — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestTotalLeastSquares : public ::testing::Test:
    TotalLeastSquares<float, 8, 1>  tls        # 8 samples, single regressor
    TotalLeastSquares<float, 10, 2> tls2        # multivariate
# each case below is a TEST_F(TestTotalLeastSquares, <name>)
```

## Test cases (Arrange / Act / Assert)

```
recovers_exact_line_no_noise:
    Arrange: y = 2x from clean data
    Act:     Fit(A, b)
    Assert:  coefficient ≈ 2

symmetric_noise_beats_ols:
    Arrange: line with equal noise added to BOTH x and y
    Assert:  TLS slope closer to truth than the OLS slope

matches_ols_when_regressors_clean:
    Arrange: noise only on b
    Assert:  TLS ≈ OLS solution (within tol)

multivariate_plane_fit:
    Arrange: b = 1.5·a1 − 0.5·a2, clean
    Assert:  coefficients ≈ [1.5, -0.5]

degenerate_returns_false:
    Arrange: b exactly in span(A) so last v-entry ≈ 0
    Assert:  Fit returns false

scaling_invariance_after_normalization:
    Arrange: columns scaled to equal norm
    Assert:  solution consistent with the unscaled reference

predict_matches_dot_product:
    Arrange: known coefficients
    Assert:  Predict(x) == dot(coeff, x)
```

## Reference vectors

- `b = 2a` clean ⇒ slope `2`, orthogonal residual `0`.
- Known 2×2 example from Golub & Van Loan (1980) ⇒ documented TLS solution.

## Edge cases

- Two smallest singular values nearly equal ⇒ ill-posed; assert detectable/flagged.
- `Samples == Features + 1` (minimal system).
- All-zero column in `A` ⇒ rank deficiency handled without divide-by-zero.
