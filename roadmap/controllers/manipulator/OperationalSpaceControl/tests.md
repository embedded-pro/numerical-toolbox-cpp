# Operational-Space Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestOperationalSpaceControl : public ::testing::Test:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>> model
    StrictMock<kinematics::MockSpatialJacobian<float,2>>     jacobian
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    OperationalSpaceControl<float,2,2> controller{ model, jacobian, Kp, Kd }
# each case below is a TEST_F(TestOperationalSpaceControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
unit_inertia_gives_task_pd:
    Arrange: model M=I, J=I, g=0, C=0; eX, eXDot given
    Act:     τ = ComputeTorque(...)
    Assert:  Λ=I ⇒ τ == xdDdot + Kd·eXDot + Kp·eX

task_inertia_computed_from_mass:
    Arrange: M = diag(2,2), J=I
    Assert:  Λ == diag(2,2); F scaled accordingly

gravity_mapped_to_task_and_back:
    Arrange: model g != 0, M=I, J=I, errors 0
    Assert:  τ == g          (Jᵀ J̄ᵀ g collapses to g)

jacobian_transpose_realises_wrench:
    Arrange: nontrivial J, known F components
    Assert:  τ == transpose(J)·F  (+ null-space term)

secondary_torque_projected_to_nullspace:
    Arrange: redundant case (Dof>TaskDim), tauSecondary != 0
    Assert:  J·M⁻¹·(N·tauSecondary) ≈ 0  (no task-space acceleration)

nullspace_orthogonality:
    Arrange: any tauSecondary
    Assert:  J · Jbar == Identity(TaskDim) and J·M⁻¹·N ≈ 0

singular_jacobian_uses_damping:
    Arrange: J rank-deficient
    Assert:  damped Λ stays finite (no NaN / inf)

msolve_and_model_terms_queried:
    Arrange: any state
    Assert:  ComputeMassMatrix, ComputeCoriolisTerms, ComputeGravityTerms each once;
             jacobian.Compute once (StrictMock)
```

## Reference vectors

- `M=I`, `J=I`: `Λ=I`, controller collapses to task-space PD `τ = Kp·eX + Kd·eXDot + xdDdot + g`.
- Consistency: `J·M⁻¹·N ≈ 0` — the null-space produces no task motion (verify numerically ≈ 0).

## Edge cases

- Redundant arm (`Dof > TaskDim`): non-trivial null space; secondary objective realised without
  disturbing the task.
- Singularity: damped inverse keeps torque bounded.
- Model mismatch: task decoupling degrades gracefully, stays stable.
