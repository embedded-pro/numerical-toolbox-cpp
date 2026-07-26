# Quaternion

## Overview & Motivation

Three-dimensional attitude representation is a fundamental requirement in robotics, aerospace,
and wearable sensing. Euler angles are intuitive but suffer from gimbal lock — a singularity
that collapses three degrees of freedom into two whenever one angle reaches ±90°. Rotation
matrices avoid this but carry nine words of state and require orthogonality re-enforcement.

A unit quaternion encodes the same rotation in four words, composes orientations with sixteen
multiply-adds, and is free of singularities. Every modern AHRS filter — Madgwick, Mahony,
Extended Kalman — stores attitude as a unit quaternion precisely because of this combination
of compactness, numerical stability, and algebraic closure.

## Mathematical Theory

### Core Definitions

A quaternion is a hypercomplex number of the form

$$q = w + x\mathbf{i} + y\mathbf{j} + z\mathbf{k}$$

where $w, x, y, z \in \mathbb{R}$ and the basis elements satisfy

$$\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{ijk} = -1.$$

A **unit quaternion** ($\|q\| = 1$) encodes a rotation by angle $\theta$ about unit axis $\hat{n}$ as

$$q = \left(\cos\frac{\theta}{2},\; \hat{n}\sin\frac{\theta}{2}\right).$$

### Hamilton Product

Composition of two rotations $q_a$ then $q_b$ is

$$q_a \otimes q_b = \begin{pmatrix}
w_a w_b - x_a x_b - y_a y_b - z_a z_b \\
w_a x_b + x_a w_b + y_a z_b - z_a y_b \\
w_a y_b - x_a z_b + y_a w_b + z_a x_b \\
w_a z_b + x_a y_b - y_a x_b + z_a w_b
\end{pmatrix}.$$

This product is **non-commutative**: $q_a \otimes q_b \neq q_b \otimes q_a$ in general.

### Vector Rotation

A pure quaternion $p = (0, \mathbf{v})$ is rotated by

$$\mathbf{v}' = q \otimes p \otimes q^{-1}.$$

For unit $q$ this simplifies (Rodrigues cross-product form) to

$$\mathbf{v}' = \mathbf{v} + 2w\,(\mathbf{u} \times \mathbf{v}) + 2\,\mathbf{u} \times (\mathbf{u} \times \mathbf{v}),$$

where $\mathbf{u} = (x, y, z)$. This costs 15 multiply-adds vs 9 for a pre-built rotation
matrix, making it preferable when rotating one vector.

### Conjugate and Inverse

For any quaternion $q^* = (w, -x, -y, -z)$. For a unit quaternion $q^{-1} = q^*$.

### Rotation Matrix

$$R(q) = \begin{pmatrix}
1-2(y^2+z^2) & 2(xy-wz)     & 2(xz+wy) \\
2(xy+wz)     & 1-2(x^2+z^2) & 2(yz-wx) \\
2(xz-wy)     & 2(yz+wx)     & 1-2(x^2+y^2)
\end{pmatrix}.$$

### Euler Angles (ZYX / 321 convention)

Converting from unit quaternion to roll $\phi$, pitch $\theta$, yaw $\psi$:

$$\phi = \operatorname{atan2}(2(wx+yz),\; 1-2(x^2+y^2))$$
$$\theta = \arcsin(2(wy-zx))$$
$$\psi = \operatorname{atan2}(2(wz+xy),\; 1-2(y^2+z^2))$$

At $\theta = \pm 90°$ the $\phi$ and $\psi$ axes align (gimbal lock); the formula still
returns a bounded value but the decomposition is no longer unique.

### SLERP

Spherical Linear Interpolation between unit quaternions $q_0$ and $q_1$ at fraction $t \in [0,1]$:

$$\operatorname{Slerp}(q_0, q_1, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega}\,q_0 + \frac{\sin(t\Omega)}{\sin\Omega}\,q_1,$$

where $\cos\Omega = q_0 \cdot q_1$. When $\Omega \approx 0$ (nearly parallel quaternions)
the formula degenerates; a normalized linear interpolation (nlerp) is substituted.

## Complexity Analysis

| Operation             | Time   | Space | Notes                                  |
|-----------------------|--------|-------|----------------------------------------|
| Hamilton product      | O(1)   | O(1)  | 16 multiply-adds, scalar only          |
| Vector rotate         | O(1)   | O(1)  | 15 multiply-adds via cross-product     |
| To rotation matrix    | O(1)   | O(1)  | 9 elements, 16 multiplications         |
| From rotation matrix  | O(1)   | O(1)  | Branch on largest diagonal             |
| SLERP                 | O(1)   | O(1)  | 1 acos + 2 sin + scalar blends         |
| Euler conversion      | O(1)   | O(1)  | 2 atan2 + 1 asin                       |

All operations are stack-only with no heap allocation.

## Step-by-Step Walkthrough

Rotating $\hat{x} = (1,0,0)$ by 90° about $\hat{z}$:

1. Axis-angle: $q = (\cos 45°,\, 0,\, 0,\, \sin 45°) = (\tfrac{\sqrt{2}}{2},\, 0,\, 0,\, \tfrac{\sqrt{2}}{2})$.
2. $\mathbf{u} = (0, 0, \tfrac{\sqrt{2}}{2})$, $\mathbf{v} = (1, 0, 0)$.
3. $\mathbf{t} = 2\,\mathbf{u} \times \mathbf{v} = 2(0 \cdot 0 - \tfrac{\sqrt{2}}{2} \cdot 0,\; \tfrac{\sqrt{2}}{2} \cdot 1 - 0,\; 0) = (0,\, \sqrt{2},\, 0)$.
4. $\mathbf{u} \times \mathbf{t} = (0 \cdot 0 - \tfrac{\sqrt{2}}{2} \cdot \sqrt{2},\; \ldots) = (-1, 0, 0)$.
5. $\mathbf{v}' = (1,0,0) + \tfrac{\sqrt{2}}{2}(0,\sqrt{2},0) + (-1,0,0) = (0,1,0) = \hat{y}$. Correct.

## Pitfalls & Edge Cases

- **Drift from unit sphere** — repeated products accumulate floating-point error; renormalize
  when $|\|q\|^2 - 1| > \varepsilon$ rather than every step.
- **Double cover** — $q$ and $-q$ represent the same rotation. SLERP flips the sign of $q_1$
  when $q_0 \cdot q_1 < 0$ to guarantee the short arc.
- **Near-parallel SLERP** — when $\cos\Omega > 0.9995$, $\sin\Omega \approx 0$ causes
  division instability; nlerp is substituted with identical results to first order.
- **Gimbal lock in ToEulerZYX** — at $\theta = \pm 90°$ the formula clamps pitch and
  returns an arbitrary roll/yaw decomposition; the rotation itself remains correct.
- **FromRotationMatrix** — branching on the largest diagonal avoids dividing by a near-zero
  value when the rotation is close to 180° about a coordinate axis.

## Variants & Generalizations

- **Dual quaternions** — extend to rigid-body transforms (rotation + translation), used in
  screw-motion interpolation.
- **Exponential map / log** — convert between the Lie algebra $\mathfrak{so}(3)$ and unit
  quaternions, enabling unbiased averaging and covariance propagation.
- **nlerp** — normalized linear interpolation is faster than SLERP but does not maintain
  constant angular velocity; acceptable for small arcs or high frame rates.

## Applications

- Attitude estimation (AHRS, IMU fusion) — the canonical state representation.
- 3D rigid-body simulation — compose joint rotations without gimbal lock.
- Animation blending — SLERP between keyframe orientations at constant angular speed.
- Computer vision — rotation parameterization in bundle adjustment and PnP solvers.

## Connections to Other Algorithms

- `Geometry3D` (`RotationAboutAxis`, `CrossProduct`) — provides the rotation matrix and
  vector primitives reused by quaternion conversions.
- Madgwick / Mahony AHRS (item 33) — propagates attitude as a unit quaternion and calls
  `operator*` / `Normalize` on every sample.
- CORDIC (item 23) — shift-add approximation of `acos`/`sin` for fixed-point axis-angle
  conversions on cores without an FPU.

## References & Further Reading

- J. B. Kuipers, *Quaternions and Rotation Sequences*, Princeton University Press, 1999.
- K. Shoemake, "Animating rotation with quaternion curves," *ACM SIGGRAPH*, 1985.
- J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors," Stanford Technical Report, 2006.
