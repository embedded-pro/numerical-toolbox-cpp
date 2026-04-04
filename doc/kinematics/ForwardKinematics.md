# Forward Kinematics

## Overview & Motivation

Forward Kinematics computes the 3D Cartesian positions of all joints in a serial kinematic chain, given the joint angles. For a chain of $n$ revolute joints, it produces $n + 1$ position vectors (base through end-effector) in $O(n)$ time.

This is the geometric foundation for visualization, collision detection, and workspace analysis. It answers the question: "given these joint angles, where is each joint in world space?"

## Mathematical Theory

### Position Computation

Starting from the base at the origin, the position of joint $i + 1$ is:

$$p_{i+1} = p_i + R_{\text{world} \to i} \cdot r_{i \to i+1}$$

where:

- $p_i$ is the position of joint $i$ in world coordinates
- $R_{\text{world} \to i} = \prod_{k=0}^{i} R_k(q_k)$ is the accumulated rotation from world to link $i$
- $r_{i \to i+1}$ is the offset from joint $i$ to joint $i + 1$ in the link $i$ frame

Each rotation $R_k(q_k)$ is computed via Rodrigues' formula from the joint axis and angle.

### End-Effector Approximation

For the last link, the offset to the end-effector is approximated as $2 \cdot r_{j \to \text{CoM}}$ (assuming the center of mass is at the midpoint of the link).

## Complexity Analysis

| Case | Time   | Space  | Notes                  |
|------|--------|--------|------------------------|
| All  | $O(n)$ | $O(n)$ | Single pass over chain |

## Variants & Generalizations

- **Planar vs. spatial chains**: In purely planar manipulators, all joint axes are parallel and rotations reduce to 2D trigonometry, but the algorithmic structure (one pass accumulating transforms) remains the same.
- **Prismatic joints**: For prismatic joints, $R_k(q_k)$ stays constant while $r_{i \to i+1}$ becomes a function of the joint displacement; the same forward sweep still applies.
- **Different parameterizations**: Denavit–Hartenberg, modified DH, or product-of-exponentials (PoE) formulations all map to the same core idea: recursively compose transforms along the chain.
- **Multiple end-effectors**: For branched chains, the same routine can be run per branch by choosing different terminal joints while reusing shared prefixes.

## Applications

- **Visualization**: Rendering joint frames and links in 3D for debugging controllers, planners, and estimators.
- **Collision detection & workspace analysis**: Computing link poses to test against environment geometry and to sample reachable workspaces.
- **Control & planning**: Providing end-effector pose and intermediate joint positions to inverse kinematics solvers, trajectory planners, and constraint checkers.
- **Dynamics algorithms**: Supplying link transforms to dynamics routines such as ABA and RNEA that require consistent kinematic states.
## Step-by-Step Walkthrough

Consider a 2-link planar arm with link lengths $L_1 = 1$, $L_2 = 0.8$, Y-axis joints, and $q = [\pi/4, -\pi/6]$.

1. $p_0 = [0, 0, 0]^T$ (base at origin)
2. $R_0 = R_y(\pi/4)$, link extent $= [1, 0, 0]^T$, so $p_1 = R_0 \cdot [1, 0, 0]^T = [0.707, 0, 0.707]^T$
3. $R_{01} = R_0 \cdot R_y(-\pi/6)$, link extent $= [0.8, 0, 0]^T$, so $p_2 = p_1 + R_{01} \cdot [0.8, 0, 0]^T$

## Pitfalls & Edge Cases

- **Zero-length links**: If `parentToJoint` and `jointToCoM` are both zero, consecutive joints collapse to the same position.
- **Floating-point only**: Uses trigonometric functions, unsuitable for fixed-point types.
- **Approximation at tip**: The end-effector position uses `jointToCoM * 2`, which is exact only for uniform-density links with the CoM at the geometric center.

## Connections to Other Algorithms

- [Articulated Body Algorithm](../dynamics/ArticulatedBodyAlgorithm.md): ABA's first pass performs a similar forward kinematics sweep to compute velocities and rotation matrices.
- [Recursive Newton-Euler](../dynamics/RecursiveNewtonEuler.md): RNEA's forward pass also propagates rotations along the chain.
- The shared rotation computation uses `math::RotationAboutAxis` from [Geometry3D](../math/Geometry3D.md).

## References & Further Reading

- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Chapters 2–3.
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Chapter 2.
