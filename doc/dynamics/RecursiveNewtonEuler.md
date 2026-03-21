# Recursive Newton-Euler Algorithm (RNEA)

## Overview & Motivation

The Recursive Newton-Euler Algorithm (RNEA) is the most efficient method for computing **inverse dynamics** of serial kinematic chains (robot arms, manipulators). Given joint positions, velocities, and desired accelerations, it computes the required joint torques in $O(n)$ time — linear in the number of links.

This contrasts with the Euler-Lagrange approach, which requires explicitly forming and multiplying $n \times n$ matrices ($O(n^3)$). For a 6-DOF robot arm, RNEA is roughly 10× faster than the matrix-based approach.

The algorithm has two passes over the kinematic chain:

1. **Forward pass** (base → tip): propagate angular velocities, angular accelerations, and linear accelerations from link to link using kinematic relationships
2. **Backward pass** (tip → base): accumulate forces and torques from the tip back to the base using Newton-Euler equations on each link

The joint torque at each joint is then the projection of the net torque onto the joint axis.

## Mathematical Theory

### Kinematic Chain Model

A serial chain of $n$ rigid bodies (links) connected by revolute joints. Each link $i$ is described by:

- $m_i$ — mass
- $I_i \in \mathbb{R}^{3 \times 3}$ — inertia tensor at the center of mass, in the link frame
- $\hat{z}_i \in \mathbb{R}^3$ — joint axis (unit vector in the link frame)
- $r_{p \to j}^{(i)} \in \mathbb{R}^3$ — position of joint $i$ in the parent (link $i-1$) frame
- $r_{j \to c}^{(i)} \in \mathbb{R}^3$ — position of the center of mass in the link $i$ frame

The rotation matrix $R_i(q_i)$ transforms vectors from link $i$ frame to parent frame, computed via Rodrigues' formula from the joint angle $q_i$ and joint axis $\hat{z}_i$.

### Forward Pass — Velocity and Acceleration Propagation

For link $i$, given parent link $i-1$ quantities:

**Angular velocity:**
$$\omega_i = R_i^T \omega_{i-1} + \dot{q}_i \hat{z}_i$$

**Angular acceleration:**
$$\dot{\omega}_i = R_i^T \dot{\omega}_{i-1} + \ddot{q}_i \hat{z}_i + \omega_i \times (\dot{q}_i \hat{z}_i)$$

**Linear acceleration at joint origin of link $i$:**
$$a_{J_i} = R_i^T \left( a_{J_{i-1}} + \dot{\omega}_{i-1} \times r_{p \to j}^{(i)} + \omega_{i-1} \times (\omega_{i-1} \times r_{p \to j}^{(i)}) \right)$$

**Linear acceleration at center of mass:**
$$a_{C_i} = a_{J_i} + \dot{\omega}_i \times r_{j \to c}^{(i)} + \omega_i \times (\omega_i \times r_{j \to c}^{(i)})$$

For the base link ($i = 0$), the "parent" is the world frame with $\omega_{-1} = 0$ and $a_{J_{-1}} = -g$ (gravity expressed as a base acceleration, a standard RNEA convention).

### Backward Pass — Force and Torque Accumulation

For each link $i$, starting from the tip:

**Net force on link $i$:**
$$f_i = m_i a_{C_i} + \sum_{\text{children } j} R_j f_j$$

**Net torque on link $i$ (about joint):**
$$\tau_i = I_i \dot{\omega}_i + \omega_i \times (I_i \omega_i) + r_{j \to c}^{(i)} \times (m_i a_{C_i}) + \sum_{\text{children } j} \left( R_j \tau_j + r_{p \to j}^{(j)} \times R_j f_j \right)$$

**Joint torque:**
$$\tau_{q_i} = \hat{z}_i^T \tau_i$$

### Rodrigues' Rotation Formula

The rotation matrix for angle $\theta$ about unit axis $\hat{u} = [u_x, u_y, u_z]^T$:

$$R = \cos\theta \cdot \mathbf{I}_3 + (1 - \cos\theta) \hat{u}\hat{u}^T + \sin\theta [\hat{u}]_\times$$

where $[\hat{u}]_\times$ is the skew-symmetric matrix of $\hat{u}$.

## Complexity Analysis

| Operation | Time | Space | Notes |
|-----------|------|-------|-------|
| Inverse dynamics | $O(n)$ | $O(n)$ | Two linear passes over $n$ links |
| Per-link forward pass | $O(1)$ | $O(1)$ | Fixed 3×3 matrix-vector operations |
| Per-link backward pass | $O(1)$ | $O(1)$ | Cross products and additions |
| Rotation matrix | $O(1)$ | $O(1)$ | Rodrigues' formula: trig functions + 3×3 matrix |

Compared to Euler-Lagrange $O(n^3)$ inverse dynamics, RNEA is dramatically faster for large $n$. For $n = 6$ (typical robot arm), RNEA performs roughly 780 floating-point operations vs. ~14,000 for the matrix approach.

## Step-by-Step Walkthrough

### Example: 1-DOF Pendulum

A single thin rod of mass $m = 2\,\text{kg}$ and length $l = 1\,\text{m}$, rotating about the z-axis. Joint at origin, CoM at $[l/2, 0, 0]^T$.

**Inertia at CoM:** $I = ml^2/12 = 0.167\,\text{kg·m}^2$ (about y and z axes)

**Setup:** $q = 0$, $\dot{q} = 0$, $\ddot{q} = 1\,\text{rad/s}^2$, zero gravity.

**Forward pass:**
- $\omega_0 = [0, 0, 0]^T + 0 \cdot [0, 0, 1]^T = [0, 0, 0]^T$
- $\dot{\omega}_0 = [0, 0, 1]^T$ (from $\ddot{q}$)
- $a_{C_0} = 0 + [0, 0, 1] \times [0.5, 0, 0] + 0 = [0, 0.5, 0]^T\,\text{m/s}^2$

**Backward pass:**
- $f_0 = 2 \cdot [0, 0.5, 0]^T = [0, 1, 0]^T\,\text{N}$
- $\tau_0 = I \cdot [0, 0, 1]^T + 0 + [0.5, 0, 0] \times [0, 1, 0]^T$
- $= [0, 0, 0.167]^T + [0, 0, 0.5]^T = [0, 0, 0.667]^T$
- $\tau_q = [0, 0, 1]^T \cdot [0, 0, 0.667]^T = 0.667\,\text{N·m}$

This matches $I_{\text{end}} \cdot \ddot{q} = \frac{ml^2}{3} \cdot 1 = 0.667$.

## Pitfalls & Edge Cases

- **Joint axis convention**: The joint axis must be a unit vector. Non-unit vectors will produce incorrect rotation matrices and torque projections.
- **Frame consistency**: All link parameters (inertia, CoM position, joint axis) must be expressed in the link's own body frame. Mixing world-frame and body-frame quantities is a common source of bugs.
- **Gravity convention**: Gravity enters as $a_0 = -g$ (negative) in the base acceleration. This "fictitious force" convention avoids special-casing gravity in each link's force calculation.
- **Serial chains only**: This implementation assumes a single serial chain with no branches. Tree-structured or closed-chain mechanisms require extensions.
- **Numerical precision**: For long chains, the recursive propagation can accumulate floating-point errors. For $n \leq 10$, this is typically negligible.
- **Fixed-point arithmetic**: Like other dynamics algorithms, physical quantities exceed Q15/Q31 range. Use `float`.

## Variants & Generalizations

| Variant | Key Difference | Use Case |
|---------|---------------|----------|
| **RNEA (this)** | $O(n)$ inverse dynamics, serial chain | Real-time robot control, computed torque |
| **RNEA + mass matrix** | Call RNEA $n+1$ times to build $M(q)$ | Forward dynamics via $\ddot{q} = M^{-1}(\tau - h)$ |
| **Articulated Body Algorithm (ABA)** | $O(n)$ forward dynamics directly | Simulation without forming $M$ |
| **Extended RNEA** | Includes friction, motor inertia | Realistic robot modeling |
| **RNEA for trees** | Handles branching chains | Humanoid robots, multi-fingered hands |

## Applications

- **Computed torque control**: RNEA provides the feedforward torque $\tau = M\ddot{q}_d + h(q, \dot{q})$ for tracking controllers at each control cycle
- **Gravity compensation**: RNEA with $\dot{q} = \ddot{q} = 0$ yields gravity torques directly
- **Simulation**: Build the mass matrix column-by-column using $n$ RNEA calls, then solve $M\ddot{q} = \tau - h$ for forward dynamics
- **Model-based friction estimation**: Compare RNEA-predicted torques with measured motor currents
- **Real-time control loops**: The $O(n)$ complexity makes RNEA suitable for 1 kHz+ control rates

## Connections to Other Algorithms

- **Newton-Euler** ([NewtonEuler.md](NewtonEuler.md)): RNEA applies the single-body Newton-Euler equations recursively across the chain. The per-link force/torque computation is identical.
- **Euler-Lagrange** ([EulerLagrange.md](EulerLagrange.md)): Both compute the same dynamics ($M\ddot{q} + C\dot{q} + g = \tau$) but RNEA is $O(n)$ vs. $O(n^3)$. RNEA is preferred for computation; Euler-Lagrange for analytical derivation.
- **Gaussian Elimination** ([GaussianElimination.md](../solvers/GaussianElimination.md)): When using RNEA for forward dynamics, the mass matrix built via RNEA is solved with Gaussian elimination.

## References & Further Reading

- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer. Chapters 3, 5.
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 7.
- Luh, J. Y. S., Walker, M. W., & Paul, R. P. C. (1980). On-line computational scheme for mechanical manipulators. *Journal of Dynamic Systems, Measurement, and Control*, 102(2), 69–76.
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson. Chapter 6.
