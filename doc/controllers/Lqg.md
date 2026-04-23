# Linear Quadratic Gaussian (LQG) Controller

## Overview & Motivation

The Linear Quadratic Gaussian (LQG) controller combines two classical optimal control results: the Linear Quadratic Regulator (LQR) and the Kalman Filter (KF). It is the optimal output-feedback controller for linear time-invariant (LTI) systems subject to Gaussian process and measurement noise.

While LQR assumes full state availability — a strict requirement rarely satisfied in practice — LQG relaxes this by estimating the state from noisy output measurements. The Kalman Filter provides the minimum-variance estimate, and LQR computes the optimal control law given that estimate. By the **Separation Principle**, these two subproblems can be solved independently and their solutions combined without loss of optimality.

LQG is widely used in aerospace guidance, robotics, and process control when sensor noise is substantial and the plant model is reasonably accurate.

## Mathematical Theory

### System Model

Consider the discrete-time LTI system:

$$x_{k+1} = A x_k + B u_k + w_k$$

$$y_k = C x_k + v_k$$

where:
- $x_k \in \mathbb{R}^n$ is the state vector
- $u_k \in \mathbb{R}^m$ is the control input
- $y_k \in \mathbb{R}^p$ is the measurement vector
- $w_k \sim \mathcal{N}(0, Q_w)$ is process noise
- $v_k \sim \mathcal{N}(0, R_v)$ is measurement noise

### LQR Subproblem

The regulator minimises the infinite-horizon quadratic cost:

$$J = \sum_{k=0}^{\infty} \left( x_k^T Q x_k + u_k^T R u_k \right)$$

The optimal state-feedback gain is:

$$K = (R + B^T P B)^{-1} B^T P A$$

where $P$ is the positive semi-definite solution to the Discrete Algebraic Riccati Equation (DARE):

$$P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A$$

The optimal control law is $u_k = -K \hat{x}_{k|k}$.

### Kalman Filter Subproblem

The optimal state estimator propagates:

**Predict:**
$$\hat{x}_{k+1|k} = A \hat{x}_{k|k} + B u_k$$

$$P_{k+1|k} = A P_{k|k} A^T + Q_w$$

**Update:**
$$K_f = P_{k|k-1} C^T (C P_{k|k-1} C^T + R_v)^{-1}$$

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_f (y_k - C \hat{x}_{k|k-1})$$

$$P_{k|k} = (I - K_f C) P_{k|k-1} (I - K_f C)^T + K_f R_v K_f^T$$

The steady-state Kalman gain converges to a fixed matrix as $k \to \infty$.

### Separation Principle

The fundamental result that makes LQG tractable is:

**Theorem (Separation Principle):** The optimal output-feedback control law for the LQG problem is the composition of (1) the optimal state estimator (Kalman Filter) and (2) the optimal state-feedback controller (LQR), designed independently.

This means the LQR gain $K$ and the Kalman gain $K_f$ do not interact — changing the noise covariances $Q_w$, $R_v$ affects only the estimator, and changing the cost weights $Q$, $R$ affects only the regulator.

### Combined Control Loop

At each time step $k$:

1. **Measurement update:** $\hat{x}_{k|k} \leftarrow \text{KF.Update}(y_k)$
2. **Compute control:** $u_k = -K \hat{x}_{k|k}$
3. **Time update:** $\hat{x}_{k+1|k} \leftarrow \text{KF.Predict}(u_k)$

## Complexity Analysis

| Step           | Time          | Space    | Notes                    |
|----------------|---------------|----------|--------------------------|
| Construction   | $O(n^3)$      | $O(n^2)$ | DARE solve for LQR gain  |
| ComputeControl | $O(n^2 + nm)$ | $O(1)$   | KF update + LQR multiply |

where $n$ = state size, $m$ = input size, $p$ = measurement size.

## Numerical Considerations

- **DARE convergence:** The LQR DARE requires $(A, B)$ to be stabilisable and $(A, Q^{1/2})$ to be detectable. Ill-conditioned cost matrices or near-unstable plants can slow convergence or yield poor numerical accuracy.
- **Kalman Filter initialisation:** The choice of initial covariance $P_0$ affects transient performance. A larger $P_0$ leads to faster initial adaptation at the cost of more initial aggressiveness.
- **Noise covariance tuning:** $Q_w / R_v$ ratio controls the balance between trusting the model (large $R_v$) versus measurements (small $R_v$). Poor tuning degrades closed-loop performance.
- **Stability:** The separation principle guarantees stability only when both the LQR gain and the Kalman Filter produce stable designs individually. Verify that the observer eigenvalues are well within the unit disc.
- **Fixed-point arithmetic:** LQG is not recommended for Q15/Q31 types unless the operating range can be normalised to $[-1, 1]$ for all state, input, and measurement signals. Use `float` unless the target platform lacks an FPU.

## Pitfalls & Edge Cases

- **Model uncertainty:** LQG is optimal only for the assumed model. Unmodelled dynamics (e.g., flexible modes, time delays) can destabilise the closed loop. LQG/LTR (loop transfer recovery) or robust variants should be considered when model fidelity is limited.
- **Non-minimum-phase systems:** LQG does not guarantee good robustness margins for non-minimum-phase plants. The loop gain can drop, resulting in poor disturbance rejection.
- **Nonlinear systems:** LQG is only optimal for linear systems with Gaussian noise. For nonlinear systems, Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF) with an LQR gain around an operating point provides a linearisation-based approximation.

## Step-by-Step Walkthrough

Consider a double integrator plant — a mass subject to force input, observed through a noisy position sensor. The discrete-time matrices with sample period $T_s = 0.1\,\text{s}$ are:

$$A = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 0.1 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

**Construction (offline):**

1. Choose LQR weights $Q = 10 I_2$, $R = 0.1$. Solve DARE for $P$, compute gain $K = (R + B^T P B)^{-1} B^T P A$.
2. Choose noise covariances $Q_w = 0.1 I_2$, $R_v = 1.0$. Solve dual DARE for steady-state Kalman gain $K_f$.
3. Verify closed-loop stability: eigenvalues of $(A - BK)$ and $(A - K_f C)$ must lie strictly inside the unit disc.

**Online loop (per sample):**

| Step | Operation                                                          | Notes                  |
|------|--------------------------------------------------------------------|------------------------|
| 1    | Receive sensor measurement $y_k$                                   | Noisy position reading |
| 2    | $\hat{x}_{k\|k} = \hat{x}_{k\|k-1} + K_f(y_k - C\hat{x}_{k\|k-1})$ | Kalman update          |
| 3    | $u_k = -K \hat{x}_{k\|k}$                                          | LQR control law        |
| 4    | $\hat{x}_{k+1\|k} = A \hat{x}_{k\|k} + B u_k$                      | Kalman predict         |

Starting from initial position $x_0 = [1, 0]^T$, the LQG regulator drives the position to zero while the Kalman Filter's estimated trajectory converges to the true state despite measurement noise.

## Variants & Generalizations

- **LQG/LTR (Loop Transfer Recovery):** A systematic method to recover the robustness margins of the full-state LQR design, which are not preserved by the LQG output-feedback loop. Achieved by scaling $Q_w \to \rho B B^T$ with $\rho \to \infty$.
- **Output-feedback MPC:** Replaces the LQR regulator with an MPC solver while retaining the Kalman Filter estimator. Handles hard state and input constraints at the cost of online optimisation.
- **LQG with integral action:** Augments the state with an integrator on the tracking error to eliminate steady-state offset for step reference signals. The augmented system $(A_a, B_a, C_a)$ is formed and the DARE solved on the expanded state.
- **Reduced-order Kalman Filter:** When $p < n$, a Luenberger observer with Kalman-optimal gain provides a simpler alternative to the full Kalman Filter.
- **Extended/Unscented LQG:** Replaces the Kalman Filter with EKF or UKF for mildly nonlinear plants while keeping the LQR gain computed around the operating point.

## Applications

- **Aerospace guidance:** Inertial navigation systems use LQG to fuse accelerometer/gyroscope measurements with GPS for optimal state estimation under sensor noise.
- **Vibration suppression:** Active damping in flexible structures (spacecraft appendages, machine tools) employs LQG to attenuate resonant modes while rejecting measurement noise from strain gauges.
- **Process control:** Chemical reactors and distillation columns use LQG when only a subset of states are measured (e.g., temperature at one point, not at every tray).
- **Robotics:** Joint torque control in manipulators with encoder noise and flexible joint dynamics benefits from LQG to smooth out the velocity estimates used by the feedback law.
- **Automotive:** Active suspension and powertrain control use LQG to balance ride comfort (low state weight on acceleration) against actuator energy (high control weight) while filtering noisy road-profile sensors.

## Connections to Other Algorithms

- **LQR:** LQG reduces to LQR when full state is directly observed (no noise, identity measurement matrix).
- **Kalman Filter:** The estimation subproblem is exactly the standard Kalman Filter.
- **MPC:** Model Predictive Control generalises LQR to handle hard constraints on states and inputs; LQG provides the estimator component in output-feedback MPC.
- **DARE Solver:** Both LQR and the steady-state Kalman Filter depend on the Discrete Algebraic Riccati Equation.
- **Separation Principle:** Connects to the Certainty Equivalence Principle in stochastic optimal control, which states that the optimal policy under Gaussian uncertainty acts as if the state estimate is the true state.

## References & Further Reading

- B. D. O. Anderson and J. B. Moore, *Optimal Control: Linear Quadratic Methods*, Prentice Hall, 1990. Chapters 7–9.
- K. J. Åström, *Introduction to Stochastic Control Theory*, Dover, 2006.
- R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems," *ASME Journal of Basic Engineering*, 82(1):35–45, 1960.
- J. C. Doyle, "Guaranteed Margins for LQG Regulators," *IEEE Transactions on Automatic Control*, 23(4):756–757, 1978. (Motivation for LQG/LTR.)
- G. F. Franklin, J. D. Powell, and M. L. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019. Chapter 9.
