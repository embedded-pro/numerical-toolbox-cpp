# Savitzky-Golay Filter — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSavitzkyGolay : public ::testing::Test:
    SavitzkyGolay<float, 5, 2, 0> sg{}     # 5-point, quadratic, smoothing
# each case below is a TEST_F(TestSavitzkyGolay, <name>)
```

## Test cases (Arrange / Act / Assert)

```
coefficients_sum_to_one:
    Arrange: Window=5, Order=2, Deriv=0
    Assert:  sum(coeffs) ≈ 1  (unity DC gain, smoothing kernel)

known_5pt_quadratic_kernel:
    Arrange: Window=5, Order=2
    Assert:  coeffs ≈ [-3, 12, 17, 12, -3] / 35

fits_polynomial_exactly:
    Arrange: input is a degree ≤ Order polynomial in n
    Assert:  output reproduces the polynomial (no bias) after warm-up

preserves_peak_height:
    Arrange: narrow Gaussian-like peak
    Act:     compare peak output vs a moving average of the same length
    Assert:  SG retains a taller peak (less flattening)

first_derivative_of_ramp_is_constant:
    Arrange: Deriv=1 kernel, linear ramp of slope m
    Assert:  output ≈ m for all samples

reset_clears_line:
    Arrange: run samples, Reset(0)
    Assert:  delay line zeroed; next output == coeffs · [0..0, x]
```

## Reference vectors

- 5-point quadratic smoothing ⇒ `[-3, 12, 17, 12, -3] / 35`.
- 7-point quadratic smoothing ⇒ `[-2, 3, 6, 7, 6, 3, -2] / 21`.

## Edge cases

- `Order = Window−1` ⇒ interpolating kernel (pass-through of the centre sample).
- Startup: outputs while the line is not yet full are documented as transient.
