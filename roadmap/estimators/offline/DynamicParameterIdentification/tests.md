# Dynamic Parameter Identification — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class MockRnea : public Rnea<float, 2>:
    MOCK_METHOD((Matrix<float,2,Params>), Regressor, (q, qd, qdd), (const, override))

class TestDynamicParameterId : public ::testing::Test:
    StrictMock<MockRnea> model
    DynamicParameterIdentification<float, 2, Params, 20> id{ model }
    Vector<Params> trueParams = {…}          # ground-truth base parameters
# each case below is a TEST_F(TestDynamicParameterId, <name>)
```

## Test cases (Arrange / Act / Assert)

```
recovers_known_parameters_noise_free:
    Arrange: model.Regressor returns Y_k;  tau_k = Y_k · trueParams
    Act:     Accumulate all samples; Estimate()
    Assert:  parameters ≈ trueParams

underexcited_trajectory_returns_false:
    Arrange: all regressor rows collinear (one posture repeated)
    Act:     Accumulate; Estimate()
    Assert:  Estimate() == false  (A not positive-definite)

reset_clears_accumulators:
    Arrange: Accumulate samples, Reset()
    Assert:  A and b zero; a fresh Estimate on new data ignores old samples

least_squares_averages_noise:
    Arrange: tau = Y·trueParams + small zero-mean noise
    Assert:  estimate within tolerance; error shrinks with more samples

accumulation_is_order_independent:
    Arrange: feed the same samples in two different orders
    Assert:  identical A, b, and estimate

single_link_scalar_case:
    Arrange: Dof = 1, one inertial parameter
    Assert:  recovers τ = I·q̈ inertia exactly
```

## Reference vectors

- 2-DOF planar arm, known `[m₁, m₂, l_c, I₁, I₂]` base set ⇒ published regressor rows reproduce τ.
- Diagonal single-parameter case ⇒ `â = mean(τ / Y)`.

## Edge cases

- Rank-deficient regressor ⇒ `Estimate()` reports failure, no divide-by-zero.
- Very stiff parameter scaling (inertia ≪ mass) ⇒ column-scale before solving.
- Zero acceleration/velocity sample ⇒ contributes only the gravity/mass columns.
