# Polynomial Point-to-Point Trajectory — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPolynomialTrajectory : public ::testing::Test:
    BoundaryConditions<float> restToRest{ .q0 = 0, .qf = 1 }   # v0=vf=0
    PolynomialTrajectory<float> cubic{ restToRest, 2.0f, Degree::Cubic }
# each case below is a TEST_F(TestPolynomialTrajectory, <name>)
```

## Test cases (Arrange / Act / Assert)

```
endpoints_match_boundary_positions:
    Assert: Sample(0).position ≈ 0  and  Sample(tf).position ≈ 1

endpoint_velocities_are_matched:
    Arrange: rest-to-rest cubic
    Assert:  Sample(0).velocity ≈ 0  and  Sample(tf).velocity ≈ 0

cubic_midpoint_is_symmetric:
    Arrange: q0=0, qf=1, tf=2 rest-to-rest
    Assert:  Sample(1.0).position ≈ 0.5  (half distance at half time)

cubic_peak_velocity_matches_formula:
    Arrange: rest-to-rest, distance d, duration tf
    Assert:  max |velocity| ≈ 1.5 * d / tf  (at midpoint)

quintic_endpoint_accelerations_are_zero:
    Arrange: quintic, a0 = af = 0
    Assert:  Sample(0).acceleration ≈ 0  and  Sample(tf).acceleration ≈ 0

nonzero_boundary_velocity_is_honoured:
    Arrange: v0 = 0.5, vf = -0.5
    Assert:  Sample(0).velocity ≈ 0.5  and  Sample(tf).velocity ≈ -0.5

sample_clamps_outside_domain:
    Assert: Sample(-1) == Sample(0)  and  Sample(tf+1) == Sample(tf)

reset_recomputes_coefficients:
    Arrange: Reset to q0=1, qf=3, tf=1
    Assert:  new endpoints reproduced exactly
```

## Reference vectors

- Rest-to-rest cubic, `q0=0, qf=1, tf=2`: `q(t) = 3(t/2)² − 2(t/2)³`; `q(1) = 0.5`, `v(1) = 0.75`.
- Peak velocity of a rest-to-rest cubic `= 1.5·(qf−q0)/tf`.

## Edge cases

- Very small `tf` ⇒ large coefficients; assert no overflow / `inf`.
- `q0 == qf` with zero boundary velocities ⇒ constant, zero velocity everywhere.
- Cubic vs quintic on the same move ⇒ quintic has smoother (bounded-jerk) acceleration.
