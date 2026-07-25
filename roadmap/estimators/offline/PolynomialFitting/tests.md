# Polynomial Least-Squares Fitting — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPolynomialFitting : public ::testing::Test:
    PolynomialFitting<float, 8, 2> fitter        # 8 samples, quadratic
# each case below is a TEST_F(TestPolynomialFitting, <name>)
```

## Test cases (Arrange / Act / Assert)

```
recovers_exact_line:
    Arrange: y = 2 + 3x sampled at 8 points (Degree 1 fitter)
    Act:     Fit(x, y)
    Assert:  coefficients ≈ [2, 3]  (within tol for T)

recovers_exact_quadratic:
    Arrange: y = 1 − 0.5x + 0.25x² on [-1,1]
    Act:     Fit; Predict at held-out points
    Assert:  predictions ≈ true polynomial

fits_noisy_data_least_squares:
    Arrange: quadratic + small zero-mean noise
    Assert:  coefficients close to truth; residual norm small

predict_uses_horner:
    Arrange: known coefficients via Fit
    Assert:  Predict(x) matches manual Horner evaluation

constant_data_gives_constant_term:
    Arrange: y = c constant
    Assert:  c0 ≈ c, higher terms ≈ 0

centering_improves_conditioning:
    Arrange: abscissa far from origin (x in [100,108])
    Assert:  centered fit matches; coefficients finite (no blow-up)

degree_zero_is_mean:
    Arrange: Degree 0 fitter
    Assert:  c0 ≈ mean(y)
```

## Reference vectors

- `y = 2 + 3x` ⇒ exact coefficients `[2, 3]`, zero residual.
- `y = x²` on a symmetric grid ⇒ `[0, 0, 1]` (odd terms vanish by symmetry).

## Edge cases

- `Samples == Degree + 1` (exact interpolation, zero residual).
- Near-collinear abscissa (repeated `x`) ⇒ singular `VᵀV`; assert graceful/pivoted handling.
