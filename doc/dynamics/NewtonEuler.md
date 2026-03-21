# Newton-Euler Dynamics

## Overview & Motivation

The Newton-Euler formulation describes the dynamics of a single rigid body by directly applying Newton's second law for translational motion and Euler's equation for rotational motion. Unlike the Euler-Lagrange approach which works in generalized (joint-space) coordinates, Newton-Euler operates in Cartesian space, computing forces and torques on individual bodies.

In the **body-fixed reference frame**, the equations are:

$$F = m(\dot{v} + \omega \times v)$$
$$\tau = I\dot{\omega} + \omega \times (I\omega)$$

Where:
- $F \in \mathbb{R}^3$ — applied force in the body frame
- $\tau \in \mathbb{R}^3$ — applied torque in the body frame
- $m$ — body mass (scalar)
- $I \in \mathbb{R}^{3 \times 3}$ — inertia tensor in the body frame
- $v, \dot{v} \in \mathbb{R}^3$ — linear velocity and acceleration in the body frame
- $\omega, \dot{\omega} \in \mathbb{R}^3$ — angular velocity and acceleration in the body frame
- $\omega \times v$ — Coriolis term due to body-frame formulation
- $\omega \times (I\omega)$ — gyroscopic coupling term

This supports two computations:
- **Forward dynamics**: given forces/torques, compute linear and angular accelerations
- **Inverse dynamics**: given desired accelerations, compute required forces/torques

## Mathematical Theory

### Newton's Second Law (Translation)

In an inertial frame, Newton's second law is simply $F = ma$. In a body-fixed (rotating) frame, the transport theorem introduces an additional term:

$$F = m(\dot{v} + \omega \times v)$$

The $\omega \times v$ term is the Coriolis acceleration that arises because the reference frame itself is rotating.

**Forward**: $\dot{v} = F/m - \omega \times v$

**Inverse**: $F = m(\dot{v} + \omega \times v)$

### Euler's Equation (Rotation)

Euler's equation governs rotational dynamics:

$$\tau = I\dot{\omega} + \omega \times (I\omega)$$

The $\omega \times (I\omega)$ term is the **gyroscopic torque** — it couples different rotational axes when the inertia tensor is non-spherical. For a body spinning about a non-principal axis, this term causes precession even without external torque.

**Forward**: $\dot{\omega} = I^{-1}(\tau - \omega \times (I\omega))$

This requires solving the $3 \times 3$ linear system $I\dot{\omega} = \tau - \omega \times (I\omega)$ via Gaussian elimination.

**Inverse**: $\tau = I\dot{\omega} + \omega \times (I\omega)$

This is a direct matrix-vector multiply plus cross product — no linear solve needed.

### Properties of the Inertia Tensor

The inertia tensor $I$ is:
- **Symmetric**: $I = I^T$ (6 independent components, not 9)
- **Positive definite**: $\omega^T I \omega > 0$ for $\omega \neq 0$
- **Constant in body frame**: For a rigid body, $I$ does not change with the body's orientation when expressed in the body frame

When $I$ is diagonal (principal axes frame), components are the **principal moments of inertia** $I_{xx}, I_{yy}, I_{zz}$, and Euler's equation simplifies to the well-known form:

$$\tau_x = I_{xx}\dot{\omega}_x + (I_{zz} - I_{yy})\omega_y\omega_z$$
$$\tau_y = I_{yy}\dot{\omega}_y + (I_{xx} - I_{zz})\omega_z\omega_x$$
$$\tau_z = I_{zz}\dot{\omega}_z + (I_{yy} - I_{xx})\omega_x\omega_y$$

## Complexity Analysis

| Operation        | Time   | Space  | Notes                                                |
|------------------|--------|--------|------------------------------------------------------|
| Forward dynamics | $O(1)$ | $O(1)$ | Fixed 3×3 system; Gaussian elimination on 3×3 matrix |
| Inverse dynamics | $O(1)$ | $O(1)$ | Matrix-vector multiply + cross products              |
| Cross product    | $O(1)$ | $O(1)$ | 6 multiplications and 3 subtractions                 |

All operations are constant-time since the spatial dimension is fixed at 3. The $3 \times 3$ Gaussian elimination is effectively a small constant cost.

## Step-by-Step Walkthrough

### Example: Uniform Sphere

**Parameters:** mass $m = 2\,\text{kg}$, radius $r = 0.5\,\text{m}$

**Inertia tensor:** $I = \frac{2}{5}mr^2 \cdot \mathbf{I}_3 = 0.2 \cdot \mathbf{I}_3$

**Forward dynamics** with $F = [6, 0, 0]^T\,\text{N}$, $\tau = [0, 0, 1]^T\,\text{N·m}$, $v = \omega = 0$:

$$\dot{v} = F/m - 0 = [3, 0, 0]^T \,\text{m/s}^2$$
$$\dot{\omega} = I^{-1}\tau = [0, 0, 5]^T \,\text{rad/s}^2$$

### Example: Gyroscopic Precession

**Asymmetric body** with $I = \text{diag}(1, 2, 3)\,\text{kg·m}^2$

Spinning at $\omega = [1, 1, 0]^T\,\text{rad/s}$ with **no external torque**:

$$I\omega = [1, 2, 0]^T$$
$$\omega \times (I\omega) = [1,1,0] \times [1,2,0] = [0, 0, 1]^T$$
$$\dot{\omega} = I^{-1}(0 - [0,0,1]) = [0, 0, -\tfrac{1}{3}]^T \,\text{rad/s}^2$$

The body accelerates about the z-axis despite no external torque — this is the gyroscopic coupling effect due to asymmetric inertia.

## Pitfalls & Edge Cases

- **Singular inertia**: An inertia tensor must be positive definite. Zero or negative eigenvalues indicate a non-physical body definition.
- **Body-frame vs. inertial frame**: The equations assume all quantities (forces, torques, velocities) are expressed in the body-fixed frame. Users must transform between frames externally.
- **Spherical inertia**: When $I = c \cdot \mathbf{I}_3$, the gyroscopic term $\omega \times (I\omega) = c(\omega \times \omega) = 0$, simplifying Euler's equation to $\tau = I\dot{\omega}$.
- **Fixed-point arithmetic**: Like Euler-Lagrange, Newton-Euler involves physical quantities (forces in Newtons, torques in N·m) that typically exceed Q15/Q31 range. Use `float`.
- **Energy conservation**: For torque-free motion, kinetic energy $T = \frac{1}{2}\omega^T I \omega$ and angular momentum $L = I\omega$ magnitude should be conserved. Numerical integration may violate this.

## Variants & Generalizations

| Variant                           | Key Difference                         | Use Case                                    |
|-----------------------------------|----------------------------------------|---------------------------------------------|
| **Newton-Euler (this)**           | Single rigid body, Cartesian space     | Spacecraft attitude, single-body simulation |
| **Recursive Newton-Euler (RNEA)** | Multi-body recursive algorithm, $O(n)$ | Robot inverse dynamics, real-time control   |
| **Euler-Lagrange**                | Generalized coordinates, joint space   | Control design, analytical derivations      |
| **Spatial vector algebra**        | 6D twist/wrench representation         | Compact multi-body formulations             |

## Applications

- **Spacecraft attitude control**: Reaction wheel and thruster control using Euler's equations
- **Drone / UAV dynamics**: Quadrotor translational and rotational dynamics
- **Projectile dynamics**: Spinning projectile with aerodynamic forces
- **Gyroscope modeling**: Precession and nutation analysis
- **Foundation for RNEA**: Body-level Newton-Euler equations form the building block for recursive multi-body algorithms

## Connections to Other Algorithms

- **Gaussian Elimination** ([GaussianElimination.md](../solvers/GaussianElimination.md)): Used to solve the $3 \times 3$ inertia system $I\dot{\omega} = \tau_{net}$ in forward dynamics
- **Euler-Lagrange** ([EulerLagrange.md](EulerLagrange.md)): Joint-space counterpart; Euler-Lagrange derives the same dynamics from energy principles in generalized coordinates
- **Recursive Newton-Euler (RNEA)**: Extends single-body Newton-Euler to kinematic chains by propagating velocities/accelerations forward and forces/torques backward through the chain

## References & Further Reading

- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 7.
- Goldstein, H., Poole, C., & Safko, J. (2002). *Classical Mechanics* (3rd ed.). Chapters 4–5.
- Hughes, P. C. (1986). *Spacecraft Attitude Dynamics*. Dover Publications.
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapter 2.
