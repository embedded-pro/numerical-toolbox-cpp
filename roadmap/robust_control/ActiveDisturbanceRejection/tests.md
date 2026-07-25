# Active Disturbance Rejection Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestActiveDisturbanceRejection : public ::testing::Test:
    # second-order plant (Order = 2): ÿ = f + b·u
    ActiveDisturbanceRejectionControl<float,2> adrc{
        /*ω_o*/ 30.0f, /*ω_c*/ 6.0f, /*b0*/ 1.0f, /*Ts*/ 0.001f }
    SecondOrderPlant plant = MakePlant(bTrue = 1.0f)
# each case below is a TEST_F(TestActiveDisturbanceRejection, <name>)
```

## Test cases (Arrange / Act / Assert)

```
estimates_total_disturbance:
    Arrange: apply a constant load f, run the ESO to steady state
    Assert:  xhat[Order] (f̂) -> f

rejects_step_disturbance:
    Arrange: closed loop, step load disturbance mid-run
    Assert:  output returns to the reference (zero steady-state error)

tracks_step_reference:
    Arrange: step reference r
    Assert:  output y -> r with no steady-state offset

eso_converges:
    Arrange: known input, no disturbance
    Assert:  estimation error y - xhat[0] -> 0

bandwidth_gain_mapping:
    Arrange: ω_o = 30, Order = 2
    Assert:  observerGain == [3ω_o, 3ω_o², ω_o³]; controlGain == [ω_c², 2ω_c]

near_model_free_robustness:
    Arrange: true plant gain bTrue != b0 within a tolerance band
    Assert:  loop stays stable and still rejects the disturbance

reset_clears_observer:
    Arrange: run, accumulate estimates, Reset()
    Assert:  xhat == 0 and appliedPrev == 0

control_cancels_disturbance_term:
    Arrange: nonzero f̂ in the estimate
    Assert:  u contains the −f̂/b0 cancellation term (correct sign/scale)
```

## Reference vectors

- `Order = 2`, `ω_o`: `β = [3ω_o, 3ω_o², ω_o³]` — golden observer gains.
- `ω_c`: `kp = ω_c²`, `kd = 2ω_c` — golden control gains (critically-damped target).

## Edge cases

- `b0` mismatch — sweep `bTrue/b0`; assert the stable/robust ratio band and document the limit.
- `ω_o` too high — noise amplification and ESO peaking; assert bounded but noisy estimate.
- Low sample rate — Euler ESO inaccuracy; assert degraded but stable behaviour is documented.
