# Backstepping Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestBacksteppingControl : public ::testing::Test:
    # Strict-feedback model injected as a StrictMock so control law is verified in isolation:
    StrictMock<MockStrictFeedbackModel<float,2>> model
    std::array<float,2> gains = { 2.0, 3.0 }        # k1, k2 > 0
    BacksteppingControl<float,2> controller{ model, gains }
# each case below is a TEST_F(TestBacksteppingControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
single_stage_reduces_to_proportional:
    Arrange: Order=1, f=0, g=1, ref=0, x=[e]
    Act:     u = ComputeControl(x, ref)
    Assert:  u == -k1 · e            (first-stage virtual control = actual input)

cancels_stage_drift:
    Arrange: Order=1, f=d, g=1, z=0
    Assert:  u == -d                 (drift cancelled exactly)

divides_by_control_gain:
    Arrange: Order=1, f=0, g=2, z=e
    Assert:  u == (-k1 · e) / 2      (gain g scales the virtual control)

two_stage_includes_cross_term:
    Arrange: Order=2; z1, z2 nonzero; g0, g1 given
    Assert:  u == ( α̇1 − f1 − k2·z2 − g0·z1 ) / g1   (Lyapunov cross term present)

virtual_control_feeds_next_stage:
    Arrange: Order=2, capture α1 for stage 0
    Assert:  z2 computed as x2 − α1   (error propagated down the chain)

reference_derivative_feedforward:
    Arrange: ref.derivative = ṙ != 0, everything else 0
    Assert:  ṙ appears in stage-0 virtual control (feedforward path)

lyapunov_decreases_closed_loop:
    Arrange: wrap an integrator-chain plant, run K steps from x0 != ref
    Assert:  V = ½Σz_i² strictly decreases each step; z -> 0

gains_set_convergence_rate:
    Arrange: two runs with k1 = 1 vs k1 = 5
    Assert:  larger gain reaches tol in fewer steps
```

## Reference vectors

- Order-1, `f=0`, `g=1`: closed loop is `ż1 = −k1·z1` ⇒ `z1[k] = z1[0]·e^(−k1·t)`, hand-computable.
- Order-2 chain of integrators: `V̇ = −k1·z1² − k2·z2² < 0` for any nonzero error.

## Edge cases

- `g_i(x) -> 0` (loss of controllability): division guard triggers documented failure/hold.
- Deadbeat-like large gains: verify no overshoot artefact from the discrete step.
- Reference with zero derivative vs ramp: feedforward term correctly absent/present.
