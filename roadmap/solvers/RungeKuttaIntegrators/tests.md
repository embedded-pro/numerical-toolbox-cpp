# Runge-Kutta ODE Integrators — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestRungeKutta : public ::testing::Test:
    # analytic references
    ScalarDecay decay{ lambda = 1 }            # ẋ = −λx,  x(t)=x0·e^(−λt)
    Harmonic    oscillator{ omega = 2 }        # ẍ = −ω²x, energy conserved
    StrictMock<OdeSystemMock<float,1,0>> systemMock    # counts Derivative() calls

    RungeKutta4<float,1,0>     rk4{ decay, 0.01 }
    DormandPrince45<float,1,0> dp45{ decay, 1e-6, 1e-6 }
# each case below is a TEST_F(TestRungeKutta, <name>)
```

## Test cases (Arrange / Act / Assert)

```
rk4_matches_exponential_decay:
    Arrange: x0 = 1, integrate ẋ=−x to t=1 with h=0.01
    Act:     step 100 times
    Assert:  x ≈ e^(−1)  (within 1e-4)

rk4_is_fourth_order:
    Arrange: integrate to fixed T with h and h/2
    Assert:  error ratio ≈ 16  (4th-order convergence)

rk4_conserves_oscillator_energy:
    Arrange: harmonic oscillator over many periods
    Assert:  energy drift bounded (no secular growth over the window)

dp45_keeps_error_below_tolerance:
    Arrange: smooth decay, absTol=relTol=1e-6
    Assert:  |x − e^(−t)| <= tolerance at each accepted step

dp45_rejects_oversized_step:
    Arrange: suggest a huge h into a fast transient
    Assert:  first StepResult.accepted == false, hNext < hUsed

dp45_grows_step_in_smooth_region:
    Arrange: long flat tail of the decay
    Assert:  hNext increases toward hMax across steps

dp45_fsal_uses_six_evaluations:
    Arrange: systemMock returning a constant derivative
    Act:     one accepted step, then a second
    Assert:  the second accepted step calls Derivative() exactly 6 times

zero_derivative_is_fixed_point:
    Arrange: f ≡ 0
    Assert:  Step returns x unchanged (both integrators)
```

## Reference vectors

- `ẋ = −x`, `x0 = 1` ⇒ `x(t) = e^(−t)` (RK4 local error `O(h⁴)`).
- Harmonic oscillator ⇒ closed-form `x(t)=x0·cos(ωt)`; energy `½(ẋ²+ω²x²)` constant.

## Edge cases

- Stiff `λ` with `h·λ > 2.8` ⇒ RK4 diverges; document the stability bound.
- `hMin` floor reached on a discontinuity ⇒ DP45 forces acceptance and flags the step.
- `h → 0` ⇒ no progress; guard against zero/negative step sizes.
