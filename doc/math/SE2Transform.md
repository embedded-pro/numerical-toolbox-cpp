# SE(2) Transform, Planar Twists and Wrenches

## Overview & Motivation

Ground robots, warehouse vehicles, mobile bases and planar mechanisms all work with poses in a plane. Odometry integrates wheel speeds into a pose, path trackers compare the vehicle pose with a reference, and landmark and scan-matching localization correct the pose from measurements. Every one of them needs the same few operations: compose two poses, invert a pose, move a velocity or a force into another frame, and turn a constant velocity held for some time into the finite motion it produces.

The planar special Euclidean group SE(2) provides exactly these. A planar pose costs four numbers instead of the twelve of a spatial transform. Composition and inversion need no trigonometry when the heading is stored as its cosine and sine.
Using the same linear-first ordering as the spatial transform keeps planar and spatial code consistent, for example when a mobile base carries an arm.

## Mathematical Theory

### Planar Transform

A transform $T = (R(\theta), p)$ with $p \in \mathbb{R}^2$ maps coordinates in frame B to frame A, $x_A = R\,x_B + p$, where

$$R(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}, \qquad T = \begin{pmatrix} R & p \\ 0 & 1 \end{pmatrix}.$$

The rotation is stored as the unit complex number $z = \cos\theta + i\sin\theta$. Composition multiplies the complex numbers and rotates the second offset, and the inverse conjugates:

$$T_1 T_2 = (z_1 z_2,\; R_1 p_2 + p_1), \qquad T^{-1} = (\bar{z},\; -R^T p).$$

The heading is never stored. It is recovered as $\theta = \operatorname{atan2}(\sin, \cos) \in (-\pi, \pi]$, so it is always wrapped.

### Ordering Convention (normative)

All 3-vectors are ordered **linear part first**: planar twists $(v_x, v_y, \omega)$, planar wrenches $(f_x, f_y, \tau)$, poses $(x, y, \theta)$, logarithms and pose errors. This matches the spatial transform's $(v;\,\omega)$ ordering.

### Adjoint, Twists and Wrenches

A twist in frame B is re-expressed in frame A by

$$V_A = \mathrm{Ad}_T V_B, \qquad \mathrm{Ad}_T = \begin{pmatrix} \cos\theta & -\sin\theta & p_y \\ \sin\theta & \cos\theta & -p_x \\ 0 & 0 & 1 \end{pmatrix}.$$

The rotation turns the linear velocity, and the offset converts the turn rate into a lever-arm velocity. The wrench map is the power-consistent dual:

$$f_A = R f_B, \qquad \tau_A = \tau_B + p_x f_{A,y} - p_y f_{A,x}.$$

Power $W \cdot V$ is then the same in every frame, and the adjoint is a homomorphism, $\mathrm{Ad}_{T_1 T_2} = \mathrm{Ad}_{T_1}\mathrm{Ad}_{T_2}$.

### Exponential

A constant twist $\xi = (v_x, v_y, \omega)$ held for a time $t$ turns by $\varphi = \omega t$ and moves along a circular arc:

$$\exp(\xi t) = \left(z(\varphi),\; V(\varphi)\,u\right), \qquad u = (v_x t, v_y t), \qquad V(\varphi) = \begin{pmatrix} A & -B \\ B & A \end{pmatrix},$$

with $A = \sin\varphi / \varphi$ and $B = (1 - \cos\varphi)/\varphi$. For $|\varphi| < 0.1$ both are replaced by their Taylor series, $A \approx 1 - \varphi^2/6 + \varphi^4/120$ and $B \approx \varphi/2 - \varphi^3/24 + \varphi^5/720$. This removes the $0/0$ at $\varphi = 0$ and the cancellation in $1 - \cos\varphi$. The truncation error at the switch is below single-precision resolution. Away from it, $B$ uses the half-angle form $2\sin^2(\varphi/2)/\varphi$.

### Logarithm

The inverse of the exponential with $t = 1$ returns the twist that carries the identity to $T$:

$$\log T = \left(V(\varphi)^{-1} p,\; \varphi\right), \qquad V(\varphi)^{-1} = \begin{pmatrix} \alpha & h \\ -h & \alpha \end{pmatrix}, \qquad h = \varphi/2, \quad \alpha = h\cot h.$$

On the branch $\varphi \in (-\pi, \pi]$, $\alpha$ is finite everywhere and reaches $0$ at $\pm\pi$, so the logarithm has no singular point. Near zero it uses $\alpha \approx 1 - \varphi^2/12 - \varphi^4/720$. Because the angle is wrapped, $\log(\exp(\xi t)) = \xi t$ only while $|\omega t| < \pi$.

### Pose Error

The base-frame pose error between a target and the current pose is

$$e = \left(p_{\text{target}} - p_{\text{current}},\; \operatorname{atan2}(\sin(\theta_t - \theta_c),\, \cos(\theta_t - \theta_c))\right).$$

The heading term is computed from the complex product $z_t \bar{z}_c$, so it always takes the short way round.

## Complexity Analysis

| Operation                     | Time   | Space  | Notes                                         |
|-------------------------------|--------|--------|-----------------------------------------------|
| Compose, invert, apply        | $O(1)$ | $O(1)$ | At most 8 multiplications, no trigonometry    |
| Adjoint, twist and wrench map | $O(1)$ | $O(1)$ | One rotation and one lever-arm term           |
| Exponential                   | $O(1)$ | $O(1)$ | One sine/cosine pair plus one half-angle sine |
| Logarithm                     | $O(1)$ | $O(1)$ | One `atan2` and one sine/cosine pair          |
| Renormalisation               | $O(1)$ | $O(1)$ | One hypotenuse                                |

## Step-by-Step Walkthrough

1. **Apply**: the pose $(1, 2, \pi/2)$ maps the point $(1, 0)$ to $(1, 3)$, a quarter turn followed by the offset. Its inverse is $(-2, 1, -\pi/2)$.
2. **Compose**: $(1, 2, \pi/2)$ composed with $(-0.5, 0.3, -\pi/3)$ gives $(0.7, 1.5, \pi/6)$. The right operand applies first.
3. **Exponential of an arc**: the unicycle twist $(1, 0, 1)$ held for $\pi/2$ traces a quarter circle of radius 1 and ends at $(1, 1, \pi/2)$. With zero turn rate, $(0.3, -0.4, 0)$ for 2 s is a straight move to $(0.6, -0.8, 0)$.
4. **Logarithm**: the pose $(1, 2, \pi/2)$ has logarithm $(3\pi/4, \pi/4, \pi/2)$. At a half turn $(1, 0, \pi)$ the logarithm stays finite at $(0, -\pi/2, \pi)$.
5. **Pose error**: from heading $0.2$ to $0.2 + 190°$, the heading error is $-170°$ ($-2.967$ rad), not $+190°$.
6. **Power invariance**: the wrench $(1.5, -0.7, 0.3)$ and the twist $(0.4, -0.2, 0.7)$ have power $0.95$ in both frames.

## Pitfalls & Edge Cases

- **Drift of the unit rotation**: each composition in single precision lets $\cos^2 + \sin^2$ drift by about one unit in the last place. A pose integrated over many steps, such as odometry, should be renormalised every step. Single compositions need no correction.
- **Wrapped logarithm**: the logarithm returns the wrapped angle. Integrators that must count full turns should integrate the exponential directly instead of round-tripping through the logarithm.
- **Half turn**: at a heading of exactly $\pi$ the branch returns $+\pi$. A sine that rounds to a tiny negative value instead yields an angle just above $-\pi$, which describes the same rotation.
- **Ordering**: twists, wrenches, logarithms and pose errors are linear first. Formulas from sources that put the angular component first must be permuted.
- **Constructed transforms**: a transform built directly from a cosine and sine must be given a unit pair. Construction from a heading always is.

## Variants & Generalizations

- **Lie-group Jacobians**: left and right Jacobians of SE(2), and the $\oplus$/$\ominus$ operators for on-manifold estimation.
- **Error-state filters**: Kalman filters that estimate a small pose perturbation in the tangent space.
- **SE(3)**: the spatial counterpart with the same conventions. A planar pose lifts to a rotation about the vertical axis.
- **Similarity group Sim(2)**: adds a scale factor for monocular mapping and image registration.

## Applications

- Wheel odometry with exact arc integration.
- Path tracking controllers that need a pose error in the vehicle frame.
- Landmark and scan-matching localization on planar maps.
- Planar mechanisms and the base pose of mobile manipulators.

## Connections to Other Algorithms

- **SE(3) Transform**: the spatial version. The planar transform lifts into it as a rotation about the vertical axis, with zero height.
- **Angle Utilities**: the shortest signed angle difference is the scalar analogue of the short-way heading error.
- **Matrix Exponential**: the general matrix exponential of the $3\times3$ twist matrix is the closed form's reference.

## References & Further Reading

- J. Solà, J. Deray, and D. Atchuthan, "A micro Lie theory for state estimation in robotics," arXiv:1812.01537, 2018 — Appendix C.
- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017 — Chapters 3 and 13.
- T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017 — Chapter 7.
