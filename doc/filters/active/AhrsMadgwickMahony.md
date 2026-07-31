# Madgwick / Mahony AHRS Filter

## Overview & Motivation

Any system that needs to know its 3-D orientation in space — a drone, a robot arm, an AR headset, a wearable device — must fuse data from multiple sensors. A **gyroscope** measures angular rate with high bandwidth and low short-term noise, but its integral drifts over time due to bias. An **accelerometer** measures the gravity vector, which gives a long-term absolute reference for pitch and roll, but it is contaminated by vibration. A **magnetometer** provides a heading reference for yaw, but it is affected by magnetic interference.

The Attitude and Heading Reference System (AHRS) filter solves the drift-correction problem in O(1) time per step with a fixed, small memory footprint. Two closely related algorithms — Madgwick's gradient-descent filter and Mahony's passive complementary filter — achieve this by continuously nudging the gyro-integrated quaternion so that the predicted sensor directions match the measured ones. Both are far cheaper to compute than a full quaternion Extended Kalman Filter, making them the standard choice for microcontroller-class attitude estimation.

## Mathematical Theory

### Quaternion State Representation

Orientation is maintained as a unit quaternion $q = [q_w, q_x, q_y, q_z]^T \in \mathbb{H}$, $\|q\| = 1$, mapping from the body frame to the Earth frame. Quaternions avoid the gimbal lock inherent in Euler angles and require fewer trigonometric operations than rotation matrices per integration step.

### Gyro Integration

The pure gyro propagation step integrates the body angular rate $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ in rad/s:

$$\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

$$q_{k+1} = q_k + \dot{q} \, T_s$$

followed by renormalization. This is the **predict** step; without a correction it drifts.

### Madgwick: Gradient-Descent Correction

Define the objective function as the alignment error between the predicted sensor directions and the measurements. For the gravity observation:

$$\mathbf{f}(q, \hat{\mathbf{a}}) = R(q)^T \mathbf{g}_{\text{ref}} - \hat{\mathbf{a}}$$

where $\mathbf{g}_{\text{ref}} = [0, 0, 1]^T$ and $\hat{\mathbf{a}}$ is the normalized accelerometer vector. The steepest-descent direction in quaternion space is:

$$\nabla F = J^T \mathbf{f}$$

where $J = \partial \mathbf{f}/\partial q$ is the $3 \times 4$ Jacobian of $\mathbf{f}$ with respect to $q$. This gradient is normalized and subtracted from the gyro-driven rate:

$$\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} - \beta \, \frac{\nabla F}{\|\nabla F\|}$$

The parameter $\beta$ is the gradient-descent step size; it is set proportional to the expected gyro measurement error in rad/s.

For the magnetometer (MARG mode), the earth frame reference is $\mathbf{b} = [b_x, 0, b_z]^T$, where $b_x$ and $b_z$ are computed by rotating the normalized magnetometer measurement into the Earth frame and zeroing its $y$-component, making the heading reference dip-angle-agnostic. A second objective function $\mathbf{f}_\text{mag}$ and its Jacobian are added to the gradient.

### Mahony: Proportional-Integral Feedback on SO(3)

Rather than gradient descent, Mahony's filter uses a cross-product error:

$$\mathbf{e} = \hat{\mathbf{a}} \times \mathbf{v}$$

where $\mathbf{v}$ is the third column of $R(q)$ (the predicted gravity direction in the body frame). The angular rate is corrected before integration:

$$\boldsymbol{\omega}_c = \boldsymbol{\omega} + K_p \mathbf{e} + \mathbf{b}_\text{est}$$

$$\dot{\mathbf{b}}_\text{est} = K_i \mathbf{e}$$

The integral term $\mathbf{b}_\text{est}$ is a running estimate of the gyro bias; once it converges, the steady-state attitude error is driven to zero even under sustained gyro drift. The proportional gain $K_p$ sets the bandwidth of the correction loop; $K_i$ sets the bias-learning rate.

For MARG mode a magnetometer cross-product error is added to $\mathbf{e}$:

$$\mathbf{e} = \hat{\mathbf{a}} \times \mathbf{v} + \hat{\mathbf{m}} \times \mathbf{w}$$

where $\mathbf{w}$ is the predicted earth-field direction in the body frame from the current tilt.

### Renormalization

Both algorithms renormalize $q$ after every integration step to enforce the unit-norm constraint, compensating for the first-order Euler integration error that would otherwise slowly push $q$ off the unit sphere.

## Complexity Analysis

| Case      | Time  | Space  | Notes                                                         |
|-----------|-------|--------|---------------------------------------------------------------|
| UpdateImu | O(1)  | O(1)   | Fixed multiply-add count; one inverse-sqrt normalization      |
| UpdateMarg| O(1)  | O(1)   | Two objective/gradient evaluations; same asymptotic cost      |
| Memory    | —     | 7 T    | 4 quaternion + 3 integral bias floats; no buffers or heap     |

The fixed cost makes both algorithms suitable for any loop rate the MCU can sustain, from 100 Hz audio-rate IMUs to 8 kHz flight-controller IMUs.

## Step-by-Step Walkthrough

**Scenario:** Quadrotor is hovering level. Gyro measures a small constant bias of 0.05 rad/s on the x-axis. Accelerometer reads $[0, 0, 9.81]$ m/s².

**Madgwick step (simplified):**

1. Normalize accelerometer: $\hat{\mathbf{a}} = [0, 0, 1]$.
2. Predicted gravity from $q \approx [1, 0, 0, 0]$: $\mathbf{v} = [0, 0, 1]$.
3. Objective: $\mathbf{f} = \mathbf{v} - \hat{\mathbf{a}} = [0, 0, 0]$ — no error, gradient is zero.
4. Rate: $\dot{q} = \frac{1}{2} q \otimes [0, \text{bias}, 0, 0]$ — small drift from bias.
5. Integrate: $q$ drifts slightly.

Over time without correction this drift accumulates; with the gradient term driving $\mathbf{f} \to 0$, Madgwick continuously nudges $q$ back to level.

**Mahony step (simplified):**

1. Cross-product error: $\mathbf{e} = [0,0,1] \times [0,0,1] = [0,0,0]$.
2. Integral accumulates: $\mathbf{b}_\text{est} \mathrel{+}= K_i \mathbf{e} \cdot T_s = 0$.
3. Corrected rate: $\boldsymbol{\omega}_c = [0.05, 0, 0] + 0 + 0 = [0.05, 0, 0]$ — still biased.
4. After the cross-product error becomes non-zero (when $q$ drifts from level), the integral term ramps up to cancel the bias, driving attitude error back to zero.

## Pitfalls & Edge Cases

- **Free-fall detection.** When $\|\mathbf{a}\| \approx 0$ (no gravity signal), the accelerometer provides no valid reference. Skipping the correction step preserves attitude at the cost of gyro drift; attempting normalization would divide by near-zero.
- **Magnetic disturbance.** Indoor environments contain ferromagnetic structures and electrical cables. When $\|\mathbf{m}\| \approx 0$ or the magnetometer reading is anomalous, falling back to 6-DOF (IMU-only) mode prevents heading corruption.
- **Beta / Kp tuning.** Too large a $\beta$ or $K_p$ leads to excessive gyro attenuation and overshoot; too small and convergence to a tilt reference is slow. The Madgwick paper recommends $\beta \approx \sqrt{3/4} \cdot \dot{\sigma}_\beta$ where $\dot{\sigma}_\beta$ is the expected gyro measurement error.
- **Quaternion sign ambiguity.** $q$ and $-q$ represent the same rotation. Algorithms that compare orientations must account for this; use the dot product $q_1 \cdot q_2 > 0$ before computing angular error.
- **Large $T_s$.** The first-order Euler integration introduces $O(T_s^2)$ error per step. At slow update rates (below ~50 Hz) higher-order integrators or additional renormalization may be needed.
- **Gimbal lock.** The quaternion representation is singularity-free; however, the Euler angle conversion $R \to (\phi, \theta, \psi)$ loses a degree of freedom at $\theta = \pm 90°$. Use the quaternion directly for any feedback control.

## Variants & Generalizations

| Variant                              | Key Difference                                                                                       |
|--------------------------------------|------------------------------------------------------------------------------------------------------|
| **6-DOF (IMU-only)**                | Accelerometer alone; roll and pitch converge, yaw is unobservable                                    |
| **9-DOF (MARG)**                    | Adds magnetometer; all three angles converge given a non-disturbed field                              |
| **Extended Kalman AHRS**            | Treats noise covariances explicitly; heavier but allows systematic tuning via $Q$/$R$ matrices       |
| **Multiplicative EKF (MEKF)**       | Kalman update on the error quaternion to preserve unit-norm; best-in-class accuracy, high cost       |
| **Gradient-descent with adaptive β**| Adjusts $\beta$ based on the magnitude of the gradient, reducing transient overshoot at startup      |
| **Second-order Runge-Kutta integration** | Reduces integration error at low update rates at the cost of one extra function evaluation     |

## Applications

- **Unmanned aerial vehicles (UAVs):** Attitude stabilization loop runs at 400–8000 Hz; the O(1) cost is critical.
- **Prosthetic limbs and rehabilitation robotics:** Accurate joint angle estimation from a wrist-worn IMU.
- **Augmented and virtual reality headsets:** Sub-millisecond latency attitude updates for display lag minimization.
- **Inertial navigation:** Dead-reckoning orientation prior to GPS fix.
- **Industrial motion capture:** Body segment tracking with arrays of MEMS IMUs.
- **Sports science wearables:** Running gait, golf swing, and rowing stroke angle analysis.

## Connections to Other Algorithms

| Algorithm                                                        | Relationship                                                                              |
|------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| [Complementary Filter](../ComplementaryFilter.md)               | The scalar 1-D ancestor; Madgwick/Mahony extend the idea to quaternion SO(3)             |
| [Extended Kalman Filter](../active/ExtendedKalmanFilter.md)     | The probabilistic alternative; heavier but allows noise covariance estimation             |
| [Quaternion](../../math/Quaternion.md)                           | The state representation shared by all three-axis attitude estimators                    |

## References & Further Reading

- Madgwick, S., "An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays," University of Bristol, 2010.
- Mahony, R., Hamel, T., Pflimlin, J.-M., "Nonlinear Complementary Filters on the Special Orthogonal Group," *IEEE Transactions on Automatic Control*, 53(5), 1203–1218, 2008.
- Diebel, J., "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors," Stanford University, 2006.
- Solin, A., Kannala, J., Rahtu, E., "Inertial Odometry on Handheld Smartphones," *FUSION 2018*.
