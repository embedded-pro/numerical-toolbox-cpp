# Lead-Lag Compensator — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestLeadLagCompensator : public ::testing::Test:
    # lead network: K=1, z=1, p=10, Ts=0.01
    LeadLagParameters<float> params{ 1.0f, 1.0f, 10.0f, 0.01f }
    LeadLagCompensator<float> comp{ params }
# each case below is a TEST_F(TestLeadLagCompensator, <name>)
```

## Test cases (Arrange / Act / Assert)

```
dc_gain_matches_continuous:
    Arrange: feed a constant, run to steady state
    Assert:  output/input ≈ K·z/p (continuous DC gain)

lead_produces_initial_overshoot:
    Arrange: lead network, unit step
    Assert:  first output > steady-state value (phase-lead kick)

lag_has_no_derivative_kick:
    Arrange: lag network (z > p), unit step
    Assert:  response is monotone toward steady state

impulse_response_is_stable:
    Arrange: unit impulse
    Assert:  output decays to 0 (|pole| < 1)

coefficients_match_tustin_design:
    Arrange: known K,z,p,Ts
    Assert:  b0,b1,a1 equal hand-computed bilinear values

reset_clears_history:
    Arrange: run samples, Reset(0)
    Assert:  next output == b0 · next_input

step_reaches_expected_steady_state:
    Arrange: unit step, lead network
    Assert:  final value ≈ K·z/p (±tol)

unity_when_zero_equals_pole:
    Arrange: z == p
    Assert:  compensator reduces to a pure gain K
```

## Reference vectors

- Continuous DC gain `C(0) = K·z/p`; the discrete steady state must match it.
- Tustin coefficients for `(K,z,p,Ts)` computed offline serve as the golden vector.

## Edge cases

- `z == p` collapses to a static gain (no dynamics).
- Very fast pole vs `Ts` (near Nyquist) — check stability and the need for pre-warping.
