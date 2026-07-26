# Sliding Mode Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestSlidingModeControl : public ::testing::Test:
    # 2-state SISO double-integrator, surface s = [c 1]·x designed for stable sliding
    math::LinearTimeInvariant<float,2,1,2> plant = MakeDoubleIntegrator()
    math::Matrix<float,1,2> S = {{ 1.0f, 1.0f }}
    math::Vector<float,1>   K = { 2.0f }
    SlidingModeControl<float,2,1> smc{ plant, S, K, 0.05f }   # φ = 0.05
# each case below is a TEST_F(TestSlidingModeControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
reaches_sliding_surface:
    Arrange: x0 off the surface, closed-loop simulate
    Assert:  |s(x)| enters the boundary layer (<= φ) in finite steps

stays_in_boundary_layer:
    Arrange: start inside |s| <= φ
    Assert:  s remains bounded by φ for all subsequent steps

equivalent_control_holds_surface:
    Arrange: x with s(x) = 0, no disturbance
    Assert:  u == u_eq and next-step s stays ≈ 0 (ṡ = 0)

rejects_matched_disturbance:
    Arrange: add matched disturbance d with |d| < K
    Assert:  surface still reached; state stays bounded

boundary_layer_suppresses_chattering:
    Arrange: run inside boundary layer, record control
    Assert:  u is continuous (Sat ramp), no ±sign flip every sample

larger_phi_increases_boundary_error:
    Arrange: solve steady state for φ1 < φ2
    Assert:  residual |s| grows with φ (monotone)

surface_gain_sets_sliding_dynamics:
    Arrange: known S on the double integrator
    Assert:  reduced-order sliding eigenvalue == designed pole (-c)

control_sign_opposes_surface:
    Arrange: s > φ (outside layer, positive)
    Assert:  switching term drives s downward (u pushes ṡ < 0)
```

## Reference vectors

- `u_eq = -(S·B)^{-1}·S·A·x` — closed form for the 2-state plant is the golden control.
- `Sat(s,φ)`: `s = 0 -> 0`, `s = φ -> 1`, `s = 2φ -> 1`, `s = -φ/2 -> -0.5`.

## Edge cases

- `φ → 0` recovers ideal sliding — document the resulting high-frequency chattering.
- Disturbance exceeding `K` — surface **not** reached; assert bounded but non-converging (documented limit).
- Singular `S·B` — invalid relative-degree design; assert precondition failure at construction.
