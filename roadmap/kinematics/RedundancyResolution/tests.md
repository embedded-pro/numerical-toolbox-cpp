# Redundancy Resolution — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestRedundancy : public ::testing::Test:
    DenavitHartenberg<float, 7>     arm{ ... }       # 7-DOF redundant arm
    SpatialJacobian<float, 7>       jac{ arm }
    RedundancyResolution<float, 7>  rr{ jac, 0.01f }
# each case below is a TEST_F(TestRedundancy, <name>)
```

## Test cases (Arrange / Act / Assert)

```
primary_task_is_satisfied:
    Arrange: desired ẋ; qdot0 = 0
    Act:     q̇ = rr.Resolve(q, ẋ, 0)
    Assert:  jac.Compute(q) * q̇ ≈ ẋ

null_space_motion_is_task_invisible:
    Arrange: arbitrary qdot0
    Act:     q̇ = P * qdot0   (task rate zero)
    Assert:  jac.Compute(q) * q̇ ≈ 0

projector_is_idempotent:
    Assert: P * P ≈ P

minimum_norm_primary_solution:
    Assert: ‖J⁺ẋ‖ ≤ ‖any other q̇ solving Jq̇ = ẋ‖

secondary_objective_reduces_cost:
    Arrange: qdot0 = -∇(joint-limit cost)
    Assert:  cost decreases along q̇ while task still met

pseudo_inverse_right_identity:
    Assert: J * J⁺ ≈ I₆  (right inverse for wide J)

damping_bounds_near_singularity:
    Arrange: near-singular q
    Assert:  ‖J⁺‖ stays finite

zero_inputs_give_zero_motion:
    Assert: Resolve(q, 0, 0) ≈ 0
```

## Reference vectors

- Wide `J` (`6×7`): `J·J⁺ ≈ I₆`, but `J⁺·J ≠ I₇` (rank 6).
- Null-space rate: `J·(P q̇₀) = 0` for any `q̇₀`.

## Edge cases

- Non-redundant arm (`N = 6`) ⇒ `P ≈ 0`, no null space.
- Fully singular `J` ⇒ damping prevents blow-up, task partially met.
- Conflicting secondary objective ⇒ primary task still exact, secondary compromised.
- Over-scaled `q̇₀` ⇒ document that the projector still protects the task.
