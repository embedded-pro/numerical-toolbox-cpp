# Linear Time-Invariant System Model

## Overview & Motivation

A **Linear Time-Invariant (LTI)** system model is the fundamental mathematical object shared across state-space control, estimation, and signal processing. It captures the dynamics of a physical plant — a motor, a pendulum, a vehicle — in a compact matrix representation that enables the systematic application of optimal control and filtering theory.

Representing an LTI model as a first-class value type makes it possible to pass a single plant description into algorithms such as the Kalman Filter, LQR, LQG, and MPC, eliminating the error-prone practice of supplying the same system matrices (A, B, C, D) separately to each component.

## Mathematical Theory

### Discrete-Time State-Space Representation

A discrete-time LTI system with state $x_k \in \mathbb{R}^n$, input $u_k \in \mathbb{R}^m$, and output $y_k \in \mathbb{R}^p$ is described by two equations:

**State equation:**
$$x_{k+1} = A x_k + B u_k$$

**Output equation:**
$$y_k = C x_k + D u_k$$

where:
- $A \in \mathbb{R}^{n \times n}$ is the **state transition matrix**
- $B \in \mathbb{R}^{n \times m}$ is the **input matrix** (sometimes called the control matrix)
- $C \in \mathbb{R}^{p \times n}$ is the **output matrix** (sometimes called the measurement matrix)
- $D \in \mathbb{R}^{p \times m}$ is the **feedthrough matrix** (often zero for strictly proper systems)

### Special Cases

**Full-state output ($C = I$, $D = 0$):** When all states are directly observable (e.g., a simulation or a state-feedback controller with perfect sensing), the output equation collapses to $y_k = x_k$. This is the typical case for LQR.

**Autonomous system ($B = 0$, $u_k = 0$):** A system driven only by initial conditions and process noise. Used in Kalman smoothing and spectral analysis.

### Discretisation

Continuous-time plants described by $\dot{x} = A_c x + B_c u$ must be discretised before use in digital controllers:

$$A = e^{A_c T_s}, \quad B = A_c^{-1}(A - I) B_c$$

where $T_s$ is the sampling period. For small $T_s$ and stable $A_c$, the forward-Euler approximation $A \approx I + A_c T_s$, $B \approx B_c T_s$ may be used.

### Stability and Controllability

An LTI model is:
- **Stable** if all eigenvalues of $A$ lie strictly inside the unit disc ($|\lambda_i| < 1$ for discrete-time systems).
- **Controllable** if the controllability matrix $\mathcal{C} = [B \; AB \; A^2B \; \cdots \; A^{n-1}B]$ has full row rank.
- **Observable** if the observability matrix $\mathcal{O} = [C^T \; (CA)^T \; \cdots \; (CA^{n-1})^T]^T$ has full column rank.

Controllability and observability are prerequisites for LQR and Kalman Filter stability, respectively.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                  |
|----------------|--------|--------|----------------------------------------|
| Step (state)   | $O(n^2 + nm)$ | $O(n)$ | One matrix-vector multiply per term   |
| Output         | $O(pn + pm)$ | $O(p)$ | One matrix-vector multiply per term    |
| Construction   | $O(1)$ | $O(n^2 + nm + pn + pm)$ | Value-type struct, no allocation |

## Numerical Considerations

- **Condition number of A:** Poorly conditioned $A$ amplifies numerical errors during iterative integration. Monitor the spectral radius of $A$ to ensure stability is not lost due to finite-precision arithmetic.
- **Feedthrough matrix D:** For most physical systems $D = 0$ (strictly proper). A nonzero $D$ requires care in closed-loop analysis since direct input-to-output coupling may cause algebraic loops.
- **Fixed-point use:** LTI matrices are suitable for Q15/Q31 when all matrix entries and intermediate products remain within $[-1, 1]$. For real physical systems with gains exceeding unity, scale the representation (e.g., normalise state and input ranges) before using fixed-point arithmetic.
- **Identity construction:** When building the $C = I$ case (full-state output), use $0.9999$ rather than $1.0$ for Q15 compatibility, since $1.0$ is not representable in the Q15 format.

## Pitfalls & Edge Cases

- **Time-varying systems:** The LTI model is invalid for systems whose matrices $A$, $B$, $C$, $D$ depend on time or operating point. Use scheduling or re-linearisation for such plants.
- **Nonlinear plants:** State-space linearisation is valid only in a neighbourhood of the operating point. Large deviations from the linearisation point invalidate the model.
- **Sampling rate:** Discrete-time LTI models are sampling-rate-specific. Changing $T_s$ requires re-discretisation and possibly re-tuning of dependent controllers and filters.

## Step-by-Step Walkthrough

Consider a double integrator (position and velocity state, force input, position output) with sample period $T_s = 0.1\,\text{s}$:

**Continuous-time plant:** $\dot{x} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} x + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u$, $y = \begin{bmatrix} 1 & 0 \end{bmatrix} x$

**Step 1 — Discretise using forward-Euler** ($A \approx I + A_c T_s$, $B \approx B_c T_s$):

$$A = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 0.1 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 \end{bmatrix}, \quad D = \begin{bmatrix} 0 \end{bmatrix}$$

**Step 2 — Simulate one step** from $x_0 = [1, 0]^T$ with $u_0 = -2$:

$$x_1 = A x_0 + B u_0 = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 1 \\ 0 \end{bmatrix} + \begin{bmatrix} 0 \\ 0.1 \end{bmatrix}(-2) = \begin{bmatrix} 1 \\ -0.2 \end{bmatrix}$$

**Step 3 — Compute output** at $k=0$:

$$y_0 = C x_0 = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} 1 \\ 0 \end{bmatrix} = 1$$

The position measurement is $1.0$ (exact, since $D = 0$ and noise is excluded here).

## Variants & Generalizations

- **Full-state output ($C = I$, $D = 0$):** All states are directly available as outputs. Used in LQR with perfect sensing. Constructed via the `WithFullStateOutput` factory, which sets $C$ to the identity.
- **Autonomous system ($B = 0$):** No input; the system evolves freely from initial conditions. Used in Kalman smoothing and spectral analysis (e.g., auto-regressive models).
- **Strictly proper systems ($D = 0$):** Most physical plants have zero feedthrough; the $D$ matrix is omitted from the signal path. The LTI representation preserves $D$ for generality.
- **Continuous-time counterpart:** Described by $\dot{x} = A_c x + B_c u$, $y = C_c x + D_c u$. Requires discretisation before use in digital controllers.
- **Multi-rate LTI:** Different sensors and actuators may operate at different sample rates; a lifted LTI representation can handle periodic multi-rate systems.

## Applications

- **Digital control design:** The LTI model is the canonical starting point for controller synthesis (LQR, MPC, pole placement). A linearised plant description is the single shared artefact that all design tools consume.
- **Kalman Filter initialisation:** The $(A, B, C)$ matrices are passed directly into the Kalman Filter to define the predict and update equations, ensuring consistency between plant and estimator.
- **Simulation and validation:** Monte Carlo simulations propagate an LTI model forward to verify closed-loop stability margins, noise sensitivity, and disturbance rejection before deployment on hardware.
- **System identification:** Identified discrete-time state-space models from input-output data (e.g., via subspace identification or prediction-error methods) are naturally expressed as LTI structs.
- **Transfer function analysis:** The $z$-domain transfer function $H(z) = C(zI - A)^{-1}B + D$ can be extracted from the LTI matrices for frequency-response computation and Bode plotting.

## Connections to Other Algorithms

- **LQR:** Uses the $(A, B)$ pair to solve the DARE and compute the optimal state-feedback gain.
- **Kalman Filter:** Uses $(A, B, C)$ to predict the next state and relate states to measurements.
- **LQG:** Composes both — $(A, B)$ for the regulator and $(A, B, C)$ for the estimator.
- **MPC:** Uses $(A, B)$ to build the prediction model over a finite horizon.
- **Kalman Smoother:** Uses $(A, C)$ for the forward-backward pass in batch smoothing.
- **DARE Solver:** Receives $(A, B)$ as inputs to compute the Riccati solution.

## References & Further Reading

- C.-T. Chen, *Linear System Theory and Design*, 4th ed., Oxford University Press, 2013. Chapters 4–5.
- R. E. Kalman, "Mathematical Description of Linear Dynamical Systems," *SIAM Journal on Control*, 1(2):152–192, 1963.
- G. F. Franklin, J. D. Powell, and A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019. Chapter 7.
- T. Kailath, *Linear Systems*, Prentice Hall, 1980. Comprehensive reference on state-space representations and transformations.
- K. J. Åström and B. Wittenmark, *Computer-Controlled Systems: Theory and Design*, 3rd ed., Dover, 2011. Chapter 3 (discretisation methods).
