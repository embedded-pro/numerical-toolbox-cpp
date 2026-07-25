# CORDIC — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCordic : public ::testing::Test:
    Cordic<float, 16> cordic
# each case below is a TEST_F(TestCordic, <name>)
```

## Test cases (Arrange / Act / Assert)

```
sincos_zero:
    Assert: SineCosine(0) ≈ { sin 0, cos 1 }

sincos_quarter_pi:
    Assert: SineCosine(π/4) ≈ { 0.7071, 0.7071 }

sincos_matches_std_over_sweep:
    Arrange: angles across [-π/2, π/2]
    Assert:  sin/cos within 2^-(Iterations-1) of std::sin/std::cos

pythagorean_identity:
    Arrange: sweep of angles
    Assert:  sin² + cos² ≈ 1

atan2_all_quadrants:
    Arrange: (y, x) in each of the four quadrants
    Assert:  Arctangent2 ≈ std::atan2 (±tol)

atan2_axes:
    Assert: atan2(0,1)=0, atan2(1,0)=π/2, atan2(0,-1)=π

magnitude_matches_hypot:
    Arrange: (3,4)-style vectors scaled into range
    Assert:  Magnitude ≈ hypot (±tol)

rotate_vector:
    Arrange: rotate (1, 0) by π/2
    Assert:  result ≈ (0, 1)

accuracy_scales_with_iterations:
    Arrange: compare Cordic<float,8> against Cordic<float,16>
    Assert:  the 16-iteration error is strictly smaller
```

## Reference vectors

- `SineCosine(π/6) = {0.5, 0.8660}`.
- `atan2(1, 1) = π/4`; `Magnitude(0.6, 0.8) = 1.0`.

## Edge cases

- Angle at the convergence-domain edge (`±π/2`) after folding.
- `atan2(0, 0)` ⇒ defined convention (return 0), never NaN.
- Minimal iteration count (`Iterations = 4`) ⇒ coarse but bounded error.
