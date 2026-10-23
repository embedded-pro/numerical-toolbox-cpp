# SE(3) Transform, Twists and Wrenches

## Overview & Motivation

Every pose-level algorithm — manipulator kinematics and Jacobians, pose inverse kinematics, Cartesian interpolation, task-space control, drone navigation, visual odometry — manipulates the same few objects: a rigid transform, a velocity-like 6-vector (a *twist*), a force-like 6-vector (a *wrench*), and the maps between a constant twist and the finite motion it produces. Defining them once, with one ordering convention, prevents the silent sign and ordering mismatches that appear when independently written modules exchange 6-vectors.

A transform is a rotation plus a translation: twelve scalars, composed with a few $3\times3$ products and inverted in closed form, with no matrix solve. A twist is re-expressed in another frame by the *adjoint*: rotate both parts, then add the lever-arm effect of the frame offset to the linear part. Wrenches follow the dual rule so that power is the same in every frame. The exponential turns "rotate about a line while sliding along it" into a finite motion; the logarithm recovers that motion, which is exactly what pose-error feedback needs.

## Mathematical Theory

### Rigid Transform

A transform $T = (R, p)$ with $R \in SO(3)$ ($R^T R = I$, $\det R = +1$) and $p \in \mathbb{R}^3$ maps coordinates in frame B to frame A, $x_A = R\,x_B + p$. Its homogeneous form is

$$T = \begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix}.$$

$$T_1 T_2 = (R_1 R_2,\; R_1 p_2 + p_1), \qquad T^{-1} = (R^T,\; -R^T p), \qquad T\cdot x = R x + p.$$

The composition applies $T_2$ first. The inverse follows from $R^{-1} = R^T$. A free direction is only rotated, $x \mapsto R x$; the translation is ignored.

### Ordering Convention (normative)

All 6-vectors are ordered **linear part first**: twists $V = (v;\, \omega)$ and wrenches $W = (f;\, n)$. This matches a geometric Jacobian with linear rows first and the tangent ordering of Sophus and manif. Lynch & Park and Murray et al. use $(\omega;\, v)$; the two conventions differ by a block permutation, so any formula from those texts converts by swapping the blocks (for the adjoint below, the rotation blocks stay on the diagonal and the $[p]_\times R$ block moves to the lower left).

### Adjoint, Twists and Wrenches

Let $[a]_\times$ be the skew matrix with $[a]_\times b = a \times b$. A twist expressed in frame B is re-expressed in frame A by

$$V_A = \mathrm{Ad}_T V_B, \qquad \mathrm{Ad}_T = \begin{pmatrix} R & [p]_\times R \\ 0 & R \end{pmatrix}, \qquad \text{i.e. } v' = R v + p \times R\omega,\;\; \omega' = R\omega.$$

The wrench map is the power-consistent dual, $\mathrm{Ad}_T^{-T}$:

$$f' = R f, \qquad n' = R n + p \times R f.$$

Power is frame invariant because the cross terms cancel by the antisymmetry of the scalar triple product:

$$W' \cdot V' = f \cdot v + n \cdot \omega + \underbrace{Rf \cdot (p \times R\omega) + (p \times Rf) \cdot R\omega}_{=\,0} = W \cdot V.$$

The adjoint is a group homomorphism, $\mathrm{Ad}_{AB} = \mathrm{Ad}_A\,\mathrm{Ad}_B$, and $\mathrm{Ad}_{T^{-1}} = \mathrm{Ad}_T^{-1}$.

### Exponential

A twist $\xi = (v;\, \omega)$ held for $\theta$ produces the finite motion $e^{[\xi]\theta}$ with $[\xi] = \begin{pmatrix} [\omega]_\times & v \\ 0 & 0 \end{pmatrix}$. It has the block form $\begin{pmatrix} e^{K\theta} & G v \\ 0 & 1 \end{pmatrix}$ with $K = [\omega]_\times$ and $G = \int_0^\theta e^{Ks}\,ds$: Rodrigues' formula for the rotation, integrated term by term for the translation. Without normalising $\omega$, and with $\varphi = \theta\,\|\omega\|$,

$$R = I + \theta\,a_1\,K + \theta^2 a_2\,K^2, \qquad p = \theta v + \theta^2 a_2\,K v + \theta^3 a_3\,K^2 v,$$

$$a_1 = \frac{\sin\varphi}{\varphi}, \qquad a_2 = \frac{1-\cos\varphi}{\varphi^2} = \frac{2\sin^2(\varphi/2)}{\varphi^2}, \qquad a_3 = \frac{\varphi - \sin\varphi}{\varphi^3}.$$

Below $\varphi = 0.1$ the coefficients switch to short Taylor series,

$$a_1 \approx 1 - \tfrac{\varphi^2}{6} + \tfrac{\varphi^4}{120}, \qquad a_2 \approx \tfrac12 - \tfrac{\varphi^2}{24} + \tfrac{\varphi^4}{720}, \qquad a_3 \approx \tfrac16 - \tfrac{\varphi^2}{120} + \tfrac{\varphi^4}{5040},$$

so there is no singular branch at $\omega = 0$ (pure translation falls out as $R = I$, $p = \theta v$) and no $0/0$ or cancellation for small $\varphi$. Because $\theta$ scales the whole twist, a non-unit $\omega$ needs no normalisation.

Re-expressing a twist in another frame generates the same motion conjugated by that frame:

$$\mathrm{Exp}(\mathrm{Ad}_A\,\xi,\, t) = A\,\mathrm{Exp}(\xi,\, t)\,A^{-1}.$$

### Logarithm of a Rotation

Let $s = \tfrac12\,\mathrm{vee}(R - R^T) = \sin\varphi\,\hat\omega$ and $\cos\varphi = \tfrac12(\operatorname{tr}R - 1)$. The angle is

$$\varphi = \mathrm{atan2}\big(\|s\|,\; \cos\varphi\big),$$

which is accurate near $0$ and $\pi$, unlike $\arccos$.

- **$\varphi \le \pi/2$:** the rotation vector is $s\,\varphi/\sin\varphi$ (zero when $R = I$).
- **$\varphi > \pi/2$:** $s/\sin\varphi$ would amplify noise as $\sin\varphi \to 0$, so the axis comes from the symmetric part, $\tfrac12(R + R^T) = \cos\varphi\, I + (1-\cos\varphi)\,\hat\omega\hat\omega^T$:

$$\hat\omega_i \hat\omega_j = \frac{\tfrac12(R_{ij} + R_{ji}) - \delta_{ij}\cos\varphi}{1 - \cos\varphi}.$$

The pivot component $i$ is the largest diagonal entry, so $\hat\omega_i^2 \ge \tfrac13$ and the divisions are benign; $\hat\omega_i = \sqrt{\hat\omega_i^2}$, the other components follow from the off-diagonal products, and the sign is taken from $s_i$. At exactly $\pi$ either sign is valid. The result satisfies $\|\omega\| \le \pi$: rotations are re-expressed the short way.

### Logarithm of a Transform

With $\omega = \mathrm{Log}(R)$ and $\varphi = \|\omega\|$, the inverse of the integral $G$ is

$$v = \Big(I - \tfrac12[\omega]_\times + c(\varphi)\,[\omega]_\times^2\Big)\,p, \qquad c(\varphi) = \frac{1 - \tfrac{\varphi}{2}\cot\tfrac{\varphi}{2}}{\varphi^2} \approx \tfrac1{12} + \tfrac{\varphi^2}{720} + \tfrac{\varphi^4}{30240} \;\;(\varphi < 0.1).$$

The result is $(v;\, \omega)$ with $\theta = 1$, so $\mathrm{Exp}(\mathrm{Log}(T), 1) = T$. The coefficient stays finite on $[0, \pi]$, reaching $1/\pi^2$ at $\varphi = \pi$.

### Pose Error

With target $(R_t, p_t)$ and current $(R_c, p_c)$, the base-frame error is

$$e = \big(p_t - p_c;\;\; \mathrm{Log}(R_t R_c^T)\big),$$

the translation difference stacked on the rotation vector that carries the current orientation onto the target. It has the linear-first ordering of a geometric Jacobian, and $\|e_\omega\| \le \pi$ always points the short way round.

## Complexity Analysis

| Operation                | Time | Space | Notes                                                                                            |
|--------------------------|------|-------|--------------------------------------------------------------------------------------------------|
| Compose                  | O(1) | O(1)  | one $3\times3$ product, one matrix–vector product                                                |
| Inverse                  | O(1) | O(1)  | transpose and one matrix–vector product; no solve                                                |
| Apply / rotate           | O(1) | O(1)  | one matrix–vector product (plus 3 additions for apply)                                           |
| Twist / wrench transform | O(1) | O(1)  | two matrix–vector products and one cross product, no $6\times6$ formed                           |
| Adjoint                  | O(1) | O(1)  | $36$ scalars, one extra $3\times3$ product; built only on request                                |
| Exp                      | O(1) | O(1)  | two sines of $\varphi$ and $\varphi/2$, none below $\varphi = 0.1$                               |
| Log                      | O(1) | O(1)  | one atan2; a half-angle sine/cosine pair above $\varphi = 0.1$; one square root for large angles |

A transform holds $12$ scalars; the $6\times6$ adjoint appears only when requested. All operations are stack-only: no heap, no recursion, constant work per call.

## Step-by-Step Walkthrough

**Apply.** $R = R_z(\pi/2)$, $p = (1,0,0)$: $R\,(1,0,0) = (0,1,0)$, plus $p$ gives $(1,1,0)$.

**Exp of a screw.** Rotation about $\hat z$ through the point $(1,0,0)$: $\omega = \hat z$, $v = -\hat z \times (1,0,0) = (0,-1,0)$, $\theta = \pi/2$.

| Step                         | Value                                                                       |
|------------------------------|-----------------------------------------------------------------------------|
| $\varphi = \theta\|\omega\|$ | $\pi/2 \ge 0.1$, direct forms                                               |
| Coefficients                 | $a_1 = 2/\pi$, $a_2 = 4/\pi^2$, $a_3 = (\pi/2 - 1)/(\pi/2)^3$               |
| Vectors                      | $Kv = \hat z \times v = (1,0,0)$, $K^2 v = (0,1,0)$                         |
| $R$                          | $\theta a_1 = 1$, $\theta^2 a_2 = 1$: $R = I + K + K^2 = R_z(\pi/2)$        |
| $p$                          | $\tfrac{\pi}{2}(0,-1,0) + (1,0,0) + (\tfrac{\pi}{2} - 1)(0,1,0) = (1,-1,0)$ |

Check: the origin sits at $(-1,0,0)$ relative to the axis; a quarter turn takes it to $(0,-1,0)$, and adding the axis point gives $(1,-1,0)$.

**Log of that transform.** $s = (0,0,1)$, $\cos\varphi = 0$, so $\varphi = \pi/2$ and $\omega = (0,0,\pi/2)$. Then $\omega \times p = (\tfrac\pi2, \tfrac\pi2, 0)$, $\omega \times (\omega \times p) = (-\tfrac{\pi^2}{4}, \tfrac{\pi^2}{4}, 0)$, and $c(\pi/2)\,\pi^2/4 = 1 - \pi/4$, so

$$v = (1 - \tfrac\pi4 - (1 - \tfrac\pi4),\; -1 - \tfrac\pi4 + (1 - \tfrac\pi4),\; 0) = (0, -\tfrac\pi2, 0).$$

The result $\big((0,-\tfrac\pi2,0);\,(0,0,\tfrac\pi2)\big)$ is $\theta\xi$, as expected.

**Pose error, short way round.** Target rotated $+190^\circ$ about $\hat z$ relative to the current pose: $\cos\varphi = \cos 190^\circ = -0.9848$, $\|s\| = \sin 170^\circ = 0.1736$, so $\varphi = 170^\circ = 2.967$ rad $> \pi/2$. The symmetric part gives $\hat\omega_z^2 = (1 + 0.9848)/1.9848 = 1$, and $s_z = \sin 190^\circ < 0$ selects $-\hat z$. The rotation error is $-170^\circ\cdot\hat z$, not $+190^\circ$.

## Pitfalls & Edge Cases

- **Ordering mismatches.** Formulas from texts using $(\omega;\, v)$ must have their blocks swapped; applying the twist adjoint to a wrench breaks power invariance, since wrenches need the dual map.
- **Drift off SO(3).** Long chains of products in single precision accumulate error in $R$. Re-orthonormalise a transform that is integrated over time, not after every single composition.
- **Non-orthonormal input to Log.** The trace and antisymmetric-part formulas assume $R \in SO(3)$; a rotation far from orthonormal gives a meaningless angle.
- **Small angles.** $\sin\varphi/\varphi$, $(1-\cos\varphi)/\varphi^2$ and $(\varphi-\sin\varphi)/\varphi^3$ are $0/0$ or lose all digits near zero; the series below $\varphi = 0.1$ remove both problems with no special case at $\omega = 0$.
- **Angles near $\pi$.** The antisymmetric part vanishes, so $s/\sin\varphi$ is unusable; the symmetric-part axis is used beyond $\pi/2$. At exactly $\pi$ the sign is arbitrary. The rotation vector is discontinuous across $\pi$ because $\pm\pi\hat\omega$ describe the same rotation.
- **Wrong logarithm.** Twice the vector part of a unit quaternion is $2\sin(\varphi/2)\,\hat\omega$, only a small-angle approximation of $\varphi\,\hat\omega$.
- **No general inverse.** Never invert the $4\times4$ matrix; the closed form is exact and cheaper.
- **Measured accuracy.** On $3000$ random twists, including $\varphi \approx 10^{-4}$ and $\varphi \in [3, 3.1405]$, Exp agrees with a double-precision $4\times4$ matrix exponential to $\approx 6\times10^{-7}$, and the Exp–Log round trip is accurate to $\approx 6\times10^{-7}$.

## Variants & Generalizations

| Variant                                    | Key difference                                                                                                     |
|--------------------------------------------|--------------------------------------------------------------------------------------------------------------------|
| **Angular-first ordering** $(\omega;\, v)$ | Same algebra with blocks permuted; used by Lynch & Park and Murray et al.                                          |
| **Body vs. spatial twist**                 | Body twists are $T^{-1}\dot T$, spatial twists $\dot T\,T^{-1}$; related by the adjoint                            |
| **Quaternion rotation part**               | Four scalars and cheap renormalisation; the rotation vector is the vector part scaled by $\varphi/\sin(\varphi/2)$ |
| **Dual quaternions**                       | Rotation and translation in one algebra; screw interpolation                                                       |
| **Similarity group**                       | Adds a uniform scale for monocular visual odometry                                                                 |

## Applications

- **Manipulator kinematics** — forward kinematics as a product of exponentials; Jacobian columns are adjoint-transformed joint twists.
- **Pose inverse kinematics and task-space control** — the pose error drives a Jacobian-based update with consistent ordering.
- **Cartesian interpolation** — Exp of a scaled Log of the relative transform gives a constant-twist screw path.
- **Force and torque sensing** — moving a measured wrench between sensor, tool and base frames with power preserved.
- **Navigation and odometry** — integrating body twists into poses on drones and in visual odometry.

## Connections to Other Algorithms

- **Quaternion** — an alternative rotation representation; renormalisation there is the cheap fix for drift, and its vector part is not the logarithm.
- **Angle Utilities** — the shortest signed angle difference is the one-dimensional analogue of the short-way rotation error: $+190^\circ$ becomes $-170^\circ$.
- **Givens Rotation** — a plane rotation embedded in a coordinate plane is the single-axis special case of the rotation block.
- **Matrix Exponential** — the general matrix exponential of the $4\times4$ twist matrix is the closed form's reference; it is used only for verification, not at run time.

## References & Further Reading

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017 — Chapter 3.
- R. M. Murray, Z. Li, and S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994 — Chapter 2.
- J. Solà, J. Deray, and D. Atchuthan, "A micro Lie theory for state estimation in robotics," arXiv:1812.01537, 2018.
