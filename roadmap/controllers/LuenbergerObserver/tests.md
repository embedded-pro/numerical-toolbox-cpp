# Luenberger Observer — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestLuenbergerObserver : public ::testing::Test:
    # 2-state SISO plant (e.g. double integrator), full-state output C = I
    math::LinearTimeInvariant<float,2,1,1> plant = MakeDoubleIntegrator()
    LuenbergerObserver<float,2,1,1> observer{ plant, DesignGain() }
# each case below is a TEST_F(TestLuenbergerObserver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
converges_to_true_state:
    Arrange: run true plant + observer from different initial conditions
    Act:     iterate Update(u, y) for K steps
    Assert:  ||xhat - xtrue|| -> 0 (below tol)

zero_innovation_when_estimate_correct:
    Arrange: seed xhat == xtrue, exact model
    Assert:  innovation == 0, estimate tracks the plant exactly

ackermann_places_requested_poles:
    Arrange: desired observer poles {0.2, 0.3}
    Act:     L = AckermannGain(plant, poles)
    Assert:  eig(A - L·C) ≈ {0.2, 0.3}

faster_poles_converge_quicker:
    Arrange: two gains with poles at 0.5 vs 0.1
    Assert:  the 0.1-pole observer reaches tol in fewer steps

rejects_initial_error_geometrically:
    Arrange: initial estimate error e0
    Assert:  error magnitude decays like the dominant observer pole per step

reset_sets_estimate:
    Arrange: Reset(x0)
    Assert:  Estimate() == x0

tracks_under_input_excitation:
    Arrange: non-zero u sequence
    Assert:  estimate follows the driven true state

feedthrough_D_accounted_in_output:
    Arrange: plant with D != 0
    Assert:  predicted output uses C·x̂ + D·u (innovation correct)
```

## Reference vectors

- Error dynamics `e[k+1] = (A − L·C) e[k]`; eigenvalues must equal the placed poles.
- Double-integrator with poles `{0.2,0.3}` gives a hand-computable `L` as the golden vector.

## Edge cases

- Unobservable `(A,C)` — Ackermann's inverse is singular; document/guard the failure.
- Deadbeat poles (all at 0) — the estimate must settle in `StateSize` steps exactly.
- Model mismatch — bounded steady-state error rather than divergence.
