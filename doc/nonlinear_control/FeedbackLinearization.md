# Feedback Linearization

## Overview & Motivation

Nonlinear plants such as robot arms, quadrotors, and electromechanical drives are only well-controlled by a fixed linear gain over a narrow operating range. Feedback linearization resolves this by exploiting a known model of the plant's nonlinearity to cancel it exactly in the closed loop, leaving an equivalent linear system — decoupled integrator chains — that a single outer-loop gain set can drive correctly across the full operating envelope. No gain scheduling, no lookup tables, no re-tuning when the operating point changes.

## Mathematical Theory

### Control-Affine Plant

The technique applies to plants whose output $y \in \mathbb{R}^m$ satisfies, after $r$ differentiations,

$$y^{(r)} = a(x) + B(x)\, u$$

where $x \in \mathbb{R}^n$ is the state, $u \in \mathbb{R}^m$ is the input, $a(x) \in \mathbb{R}^m$ is the **drift term** (known nonlinear dynamics), and $B(x) \in \mathbb{R}^{m \times m}$ is the **decoupling matrix** (state-dependent input gain). The integer $r$ is the relative degree. For mechanical systems ($r = 2$), written as $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = u$, the control-affine form has $B(x) = M^{-1}(q)$ and $a(x) = -M^{-1}(q)\bigl(C(q,\dot{q})\dot{q} + g(q)\bigr)$.

### Inner Control Law (Cancellation)

The inner law selects $u$ so that the term $a(x)$ is cancelled and the decoupling matrix is factored out:

$$u = B^{-1}(x)\,(v - a(x))$$

Substituting into the plant equation yields

$$y^{(r)} = a(x) + B(x)\,B^{-1}(x)\,(v - a(x)) = a(x) + (v - a(x)) = v$$

leaving pure integrator chains $y^{(r)} = v$, provided $B(x)$ is nonsingular.

### Outer Control Law (Linear Outer Loop)

With the plant reduced to integrators, a PD outer loop commands the virtual input:

$$v = y_d^{(r)} + K_d\,\dot{e} + K_p\, e, \quad e = y_d - y, \quad \dot{e} = \dot{y}_d - \dot{y}$$

The closed-loop error satisfies the linear ODE

$$e^{(r)} + K_d\,\dot{e} + K_p\, e = 0$$

whose eigenvalues are set by choosing $K_p, K_d$. Critical damping per channel requires $K_d = 2\sqrt{K_p}$.

### Combined Law

Expanding yields the single expression evaluated on the hot path:

$$u = B^{-1}(x)\bigl(y_d^{(r)} + K_d\,\dot{e} + K_p\, e - a(x)\bigr)$$

The law requires solving the linear system $B(x)\,u = v - a(x)$ on the hot path; $B(x)$ must be nonsingular.

## Complexity Analysis

| Operation    | Time               | Space        | Notes                                  |
|--------------|--------------------|--------------|----------------------------------------|
| Construction | $O(m^2)$           | $O(m^2)$     | Copy two gain matrices                 |
| ComputeInput | $O(m^3)$           | $O(m)$ extra | One linear solve dominates             |
| Model query  | $O(m^2)$–$O(nm^2)$ | $O(m^2)$     | Implementation-defined; injected model |

All storage is in fixed-size stack arrays; the law itself performs no heap allocation.

## Step-by-Step Walkthrough

Consider a 2-DOF planar arm with $m = 2$, $K_p = 100 I$, $K_d = 20 I$, and at one instant:

- State $x = [0.1, 0.2]^\top$, $\dot{x} = [0, 0]^\top$.
- Reference $y_d = [0.5, 0.5]^\top$, $\dot{y}_d = [0, 0]^\top$, $\ddot{y}_d = [0, 0]^\top$.
- Model returns $B(x) = I$ and $a(x) = [0.3, 0.1]^\top$.

1. Compute error: $e = [0.4, 0.3]^\top$, $\dot{e} = [0, 0]^\top$.
2. Compute virtual input: $v = 0 + 20 \cdot 0 + 100 \cdot [0.4, 0.3]^\top = [40, 30]^\top$.
3. Inner law: $u = I^{-1}([40, 30]^\top - [0.3, 0.1]^\top) = [39.7, 29.9]^\top$.

The gravity-like drift $a(x)$ is added directly; the outer PD term drives position error to zero.

## Pitfalls & Edge Cases

- **Singular decoupling matrix**: if $B(x)$ is rank-deficient the inner law is undefined. The condition $\det B(x) \neq 0$ must hold throughout the operating region.
- **Model mismatch**: cancellation is only as exact as the model. Unmodelled dynamics or parameter error leaves a residual nonlinearity; pair with a robust or adaptive outer term to bound the error.
- **Zero dynamics**: exact linearisation of the output may leave internal states unobservable. These zero dynamics can be unstable even when the output tracks perfectly. Verify stability of the internal dynamics before deployment.
- **Actuator limits**: the inner law can command arbitrarily large $u$ near the start of a transient. Saturation on $u$ breaks the exact cancellation argument; scale $K_p$, $K_d$ or add a reference pre-filter to keep the command within actuator bounds.
- **Float precision**: for large $m$, matrix products accumulate rounding error proportional to $m \cdot \epsilon_\text{float}$. Verify the gain matrices are well-conditioned.

## Variants & Generalizations

- **Input-output linearization (SISO)**: for scalar output with relative degree $r > 1$, the cancellation uses Lie derivatives $L_f^r h(x)$ and $L_g L_f^{r-1} h(x)$, and the input is $u = (v - L_f^r h(x)) / L_g L_f^{r-1} h(x)$. The singularity condition $L_g L_f^{r-1} h \neq 0$ replaces $\det B \neq 0$.
- **Computed-torque control**: the mechanical specialisation with $B = M(q)$ and $a = C(q,\dot{q})\dot{q} + g(q)$. The canonical instantiation lives in robotics-toolbox-cpp.
- **Partial feedback linearization**: linearizes only the input-output channels, leaving the rest of the state dynamics (zero dynamics) uncontrolled by the outer loop.
- **Adaptive feedback linearization / MRAC**: replaces the fixed model with an online-adapted estimate, enabling cancellation under parametric uncertainty.

## Applications

- Robot manipulators: decoupled Cartesian impedance or position control across the full joint-space workspace.
- Quadrotor UAVs: attitude and altitude decoupling for independent channel control.
- Electromechanical drives: cancellation of back-EMF and friction in torque-controlled axes.
- Chemical process control: inversion of Hammerstein-type nonlinear input maps.

## Connections to Other Algorithms

- **Backstepping**: recursive alternative for strict-feedback systems; tolerates drift terms that cannot be directly cancelled.
- **Model Reference Adaptive Control (MRAC)**: adapts the model online; complements feedback linearization when the plant parameters are unknown.
- **LQR**: natural choice for the outer linear loop once the plant has been linearized.
- **Sliding Mode Control**: robustifies the outer loop against residual model mismatch by adding a discontinuous reaching term.

## References & Further Reading

- A. Isidori, *Nonlinear Control Systems*, 3rd ed., Springer, 1995.
- J.-J. Slotine, W. Li, *Applied Nonlinear Control*, Prentice-Hall, 1991, Chapter 6.
- H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice-Hall, 2002, Chapter 13.
