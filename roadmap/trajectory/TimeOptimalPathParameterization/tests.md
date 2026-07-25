# Time-Optimal Path Parameterization (TOPP) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestTopp : public ::testing::Test:
    StrictMock<PathGeometryMock<float, 2>> path      # DI: q(s), q'(s), q''(s)
    StrictMock<JointLimitsMock<float, 2>>  limits     # DI: RNEA coeffs + bounds
    TimeOptimalPathParameterization<float, 2, 64> topp{ path, limits }
# each case below is a TEST_F(TestTopp, <name>)
```

## Test cases (Arrange / Act / Assert)

```
straight_path_respects_velocity_limit:
    Arrange: linear q(s), velocity limit vMax
    Act:     Parameterize(); sample ṡ along path
    Assert:  every joint velocity |q'(s)·ṡ| ≤ vMax

profile_respects_torque_bounds:
    Arrange: mock RNEA coeffs a,b,c with symmetric torque limits
    Assert:  reconstructed τ_j(s) ∈ [τ_min, τ_max] at all gridpoints

starts_and_ends_at_rest:
    Assert: ṡ(0) ≈ 0  and  ṡ(1) ≈ 0  (x_0 = x_N = 0)

is_faster_than_uniform_traversal:
    Arrange: same path, fixed slow ṡ
    Assert:  TotalTime() < uniform-traversal time (optimality sanity check)

infeasible_path_reports_failure:
    Arrange: torque bounds too tight to move
    Assert:  Parameterize() == false

bang_bang_structure_on_simple_path:
    Arrange: single-joint, constant inertia
    Assert:  accel is at a bound almost everywhere (max-accel then max-decel)

denser_grid_refines_time:
    Arrange: Grid 32 vs 128
    Assert:  TotalTime() converges (monotone, bounded change)

sample_before_parameterize_returns_nullopt:
    Assert: Sample(t) == nullopt until Parameterize() succeeds
```

## Reference vectors

- Single joint, unit inertia, `|τ| ≤ 1`, path length `L`, rest-to-rest ⇒ bang-bang time `2√L`.
- Velocity-limited straight segment ⇒ `ṡ_max = vMax / |q'(s)|` on the flat portion.

## Edge cases

- Zero-length path (`q(s)` constant) ⇒ `TotalTime() == 0`, trivial success.
- MVC singularity where `q'_j(s) = 0` ⇒ no division blow-up, profile stays finite.
- Velocity-only vs torque-only limiting ⇒ correct binding constraint selected per gridpoint.
