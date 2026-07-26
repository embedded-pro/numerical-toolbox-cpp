# Impedance Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestImpedanceControl : public ::testing::Test:
    # both injected dependencies mocked; TaskDim = 2 (planar xy point), Dof = 2:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    StrictMock<kinematics::MockSpatialJacobian<float,2>>     jacobian
    SquareMatrix<float,2> Md     = diag(1, 1)
    SquareMatrix<float,2> Dd     = diag(10, 10)
    SquareMatrix<float,2> Kstiff = diag(500, 500)
    ImpedanceControl<float,2,2> controller{ model, jacobian, Md, Dd, Kstiff }
# each case below is a TEST_F(TestImpedanceControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
stiffness_pulls_toward_target:
    Arrange: jacobian J=I, model terms 0; eX != 0, velocities 0, fExt 0
    Assert:  τ == Kstiff · eX

damping_opposes_task_velocity:
    Arrange: J=I, qDot != 0 (so xDot != 0), eX 0, xdDot 0
    Assert:  τ contribution == -Dd · xDot

inertia_feedforward_applied:
    Arrange: xdDdot != 0, other terms 0
    Assert:  F includes Md · xdDdot

external_force_passthrough:
    Arrange: fExt != 0, all errors 0, J=I
    Assert:  τ == fExt          (Jᵀ = I)

jacobian_transpose_maps_wrench:
    Arrange: J = [[1,0],[0,2]], known F, model 0
    Assert:  τ == transpose(J) · F

gravity_and_coriolis_added:
    Arrange: model g, Cq̇ nonzero; F = 0
    Assert:  τ == Cq̇ + g

zero_error_no_contact_holds_gravity:
    Arrange: all errors 0, fExt 0
    Assert:  τ == Cq̇ + g

both_dependencies_queried:
    Arrange: any state
    Assert:  jacobian.Compute called once; model gravity + Coriolis each once;
             ComputeMassMatrix never called (StrictMock)
```

## Reference vectors

- Pure spring (`J=I`, model 0): `F = Kstiff·eX`, `τ = JᵀF` — hand-checkable.
- Contact equilibrium: at steady state `Kstiff·eX = −fExt` ⇒ `eX_ss = −Kstiff⁻¹·fExt` (rendered compliance).

## Edge cases

- Near-singular `J`: `Jᵀ` stays finite, torque bounded (no inversion) — the key robustness property.
- Very high `Kstiff`: approaches rigid position control; watch contact instability vs sample rate.
- Noisy `fExt`: `Dd` filters the response; document the sensor bandwidth.
