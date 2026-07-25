# Hybrid Position/Force Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestHybridPositionForceControl : public ::testing::Test:
    # both dependencies mocked; planar TaskDim = 2, Dof = 2:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    StrictMock<kinematics::MockSpatialJacobian<float,2>>     jacobian
    SquareMatrix<float,2> S  = diag(1, 0)          # axis 0 = motion, axis 1 = force
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    SquareMatrix<float,2> Kf = diag(  2,   2)
    SquareMatrix<float,2> Ki = diag(  5,   5)
    float dt = 0.001
    HybridPositionForceControl<float,2,2> controller{ model, jacobian, S, Kp, Kd, Kf, Ki, dt }
# each case below is a TEST_F(TestHybridPositionForceControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
motion_axis_does_position_pd:
    Arrange: J=I, model 0; only axis-0 position error, force error 0
    Assert:  F[0] == Kp[0]·eX[0] + Kd[0]·eXDot[0];  axis-1 unaffected

force_axis_does_force_pi:
    Arrange: J=I, model 0; only axis-1 force error, position error 0, one step
    Assert:  F[1] == fd[1] + Kf[1]·eF[1] + Ki[1]·eF[1]·dt

selection_partitions_axes:
    Arrange: S = diag(1,0), both motion and force commands nonzero
    Assert:  F[0] from motion loop, F[1] from force loop (no cross-talk)

complementary_projectors_orthogonal:
    Arrange: verify S·(I−S) == 0 and S·S == S
    Assert:  each axis contributes exactly one loop

force_integral_accumulates:
    Arrange: constant force error, call ComputeTorque N times
    Assert:  integral term grows as Ki·eF·(N·dt)

reset_clears_force_integral:
    Arrange: accumulate integral, Reset()
    Assert:  next force term has no integral history

jacobian_transpose_maps_wrench:
    Arrange: J = [[1,0],[0,2]], known F
    Assert:  τ == transpose(J)·F  (+ model terms)

gravity_and_coriolis_added:
    Arrange: model g, Cq̇ nonzero; F = 0
    Assert:  τ == Cq̇ + g

both_dependencies_queried:
    Arrange: any state
    Assert:  jacobian.Compute once; model gravity + Coriolis each once;
             ComputeMassMatrix never called (StrictMock)
```

## Reference vectors

- `S=diag(1,0)`, `J=I`, model 0: `F = [Kp·eX[0], fd[1]+Kf·eF[1]+Ki·∫eF[1]]`, `τ = JᵀF` — hand-checkable.
- Contact equilibrium on the force axis: PI drives `fMeasured → fd` ⇒ steady `eF = 0`.

## Edge cases

- Contact loss (`fMeasured → 0`): force integral winds up ⇒ `Reset()` / anti-windup required.
- `S = I` (all motion) reduces to pure Cartesian PD; `S = 0` (all force) to pure force control.
- Near-singular `J`: `Jᵀ` stays finite, torque bounded (no inversion).
