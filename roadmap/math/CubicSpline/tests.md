# Cubic Spline Interpolation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestCubicSpline : public ::testing::Test:
    using Spline = CubicSpline<float, 3, 8>
    std::array<float, 8>                 knots{ 0, 1, 2.5, 3, 4.5 }        # K = 5 used
    std::array<math::Vector<float,3>, 8> values{ (0, 0.5, 0), (1, 0.2, 0), (−0.5, 0.2, 0),
                                                 (0.3, −0.4, 0), (2, 0, 0) }
    Spline spline
    void SetUp(): ASSERT_TRUE(spline.Fit(knots, values, 5))      # clamped, zero end slopes
# each case below is a TEST_F(TestCubicSpline, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
passes_through_every_knot:
    Assert: Evaluate(x_k).value ≈ y_k for k = 0..4, every channel

first_and_second_derivatives_continuous_at_interior_knots:
    Assert: for k = 1..3: Evaluate(x_k − δ) ≈ Evaluate(x_k + δ) in first and second derivative
            (δ = 1e-4, tol 1e-2 ≥ |y'''|·2δ), every channel

knot_second_derivatives_match_reference:
    Assert: channel 0: Evaluate(x_k).secondDerivative ≈ (5.665169, −5.330337, 5.991011, −0.737079, −1.898127);
            channel 1: ≈ (−1.666292, 1.532584, −2.797753, 3.384270, −2.225468);
            channel 2 (all knots 0) ≡ 0

clamped_end_slopes_are_honoured:
    Arrange: Fit(..., { Clamped, startSlope = (0.4, 0, 0), endSlope = (−0.7, 0, 0) })
    Assert:  Evaluate(0).firstDerivative_0 ≈ 0.4, Evaluate(4.5).firstDerivative_0 ≈ −0.7;
             Evaluate(0).secondDerivative_0 ≈ 4.296629, Evaluate(4.5).secondDerivative_0 ≈ −3.637453

natural_ends_have_zero_second_derivative:
    Arrange: Fit(..., { Natural })
    Assert:  Evaluate(0).secondDerivative ≈ 0, Evaluate(4.5).secondDerivative ≈ 0;
             channel 0 end slopes ≈ 1.680287 and 0.783154

collinear_knots_reproduce_straight_line:
    Arrange: knots {0, 0.5, 1, 1.5, 2}, y_k = 1 + 0.8·x_k (all channels), Clamped with slope 0.8
    Assert:  Evaluate(0.7) ≈ { 1.56, 0.8, 0 };  second derivative ≈ 0 everywhere sampled

three_knot_reference_values:
    Arrange: knots {0, 1, 2}, y {0, 1, 0}, clamped zero
    Assert:  Evaluate(0.5) ≈ { 0.5, 1.5, 0 };  knot second derivatives (6, −6, 6);  Evaluate(1).firstDerivative ≈ 0

evaluate_clamps_outside_domain:
    Assert: Evaluate(−1) == Evaluate(0)  and  Evaluate(5.5) == Evaluate(4.5)

fit_rejects_invalid_knots:
    Assert: Fit(count = 1) == false;  Fit(count = 9) == false;  knots {0, 1, 1, 2} == false
```

## Reference vectors

Computed with python3 (tridiagonal system + Thomas solve, checked against the defining conditions).

- Fixture channel 0 (`x = 0, 1, 2.5, 3, 4.5`; `y = 0, 1, −0.5, 0.3, 2`):
  - clamped zero ⇒ `M = (5.665169, −5.330337, 5.991011, −0.737079, −1.898127)`;
  - clamped `(0.4, −0.7)` ⇒ `M = (4.296629, −4.993258, 5.779775, −0.058427, −3.637453)`;
  - natural ⇒ `M = (0, −4.081720, 5.605735, −1.400717, 0)`, end slopes `1.680287`, `0.783154`.
- Fixture channel 1 (`y = 0.5, 0.2, 0.2, −0.4, 0`), clamped zero ⇒
  `M = (−1.666292, 1.532584, −2.797753, 3.384270, −2.225468)`.
- Three knots `(0,0), (1,1), (2,0)`, clamped zero ⇒ `M = (6, −6, 6)`, `y(0.5) = 0.5`, `y'(0.5) = 1.5`;
  natural ⇒ `M = (0, −3, 0)`, `y(0.5) = 0.6875`.
- Collinear knots with matching clamped slope ⇒ right-hand side ≡ 0 ⇒ `M ≡ 0`.

## Edge cases

- `K = 2`, Natural ⇒ `M = 0`: straight line (end slopes free).
- Unequal spacing (fixture) — the formulation never assumes uniform `h`.
- `K = MaxKnots` (8) ⇒ accepted; no out-of-bounds access in the binary search at `x = x_{K−1}`.
