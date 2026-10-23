# Levenberg–Marquardt (Damped Gauss–Newton with Retraction)

## Overview & Motivation

Inverse kinematics, forward kinematics of parallel robots, sensor and hand-eye calibration and scan matching all ask the same question: which state makes a set of nonlinear residuals as small as possible? The problems are small, solved every control cycle or at commissioning, and must finish within a hard iteration budget without heap allocation.

**Levenberg–Marquardt** answers it by repeated linearization. Near the current guess the residuals behave like a linear function of a small correction, and Gauss–Newton solves that linear problem exactly, converging quadratically when a zero residual is reachable. Far from the solution, or near a singular Jacobian, the exact step can be huge or undefined. A **damping** term penalizes large corrections and trades speed for robustness.

The state need not be a plain vector: an angle wraps, an orientation lives on a manifold, a joint stops at a limit. The solver therefore never adds the correction itself. The problem applies it through a **retraction**, so the state never leaves its valid set. One careful solver replaces a hand-rolled Newton loop in every module.

## Mathematical Theory

### Problem Statement

$$
\min_{x}\; \tfrac12\,\|r(x)\|^2, \qquad r(x) \in \mathbb{R}^{R}
$$

The state $x$ may lie on a manifold or within bounds and has $P$ local degrees of freedom. The problem supplies three things:

- the residual $r(x)$;
- the Jacobian with respect to a local increment, $J = \partial r(x \oplus \delta)/\partial\delta\,\big|_{\delta=0} \in \mathbb{R}^{R\times P}$;
- the retraction $x \oplus \delta$: plain addition for vectors, addition followed by clamping for bounded states, a manifold update (a rotation composed with the exponential of $\delta$) for orientations. It must agree with the exact motion to first order.

A damping policy supplies $\lambda^2 \ge 0$ from the current Jacobian. A constant policy is classic Levenberg–Marquardt, and $\lambda = 0$ is Gauss–Newton. A problem-aware schedule, such as zero away from singularities, plugs in through the same interface.

### Damped Step

Linearizing $r(x \oplus \delta) \approx r + J\delta$ and penalizing the correction gives

$$
\min_{\delta}\; \tfrac12\|r + J\delta\|^2 + \tfrac12\lambda^2\|\delta\|^2 \;\Longrightarrow\; (J^\top J + \lambda^2 I)\,\delta = -J^\top r .
$$

The push-through identity $(J^\top J + \lambda^2 I)^{-1}J^\top = J^\top (JJ^\top + \lambda^2 I)^{-1}$ gives an equivalent form, so the solver always factors the smaller system:

| Shape          | Condition | System                        | Step                                              |
|----------------|-----------|-------------------------------|---------------------------------------------------|
| Tall           | $R > P$   | $P \times P$, parameter space | $\delta = -(J^\top J + \lambda^2 I)^{-1}J^\top r$ |
| Wide or square | $R \le P$ | $R \times R$, residual space  | $\delta = -J^\top (JJ^\top + \lambda^2 I)^{-1} r$ |

For $\lambda = 0$ and full row rank, the wide form is the minimum-norm solution of $J\delta = -r$: the shortest correction that cancels the linearized residual. The system is solved by LU decomposition with partial pivoting, never by an explicit inverse. If the factorization finds a pivot below a relative $10^{-6}$ (for example $\lambda = 0$ with a rank-deficient $J$), the solve ends unconverged at the current state instead of dividing by zero. For $\lambda^2 > 0$ the matrix is symmetric positive definite.

### Iteration

With tolerance $\varepsilon$, optional step cap $\Delta$ and iteration budget $N$:

1. Evaluate $r$. If $\|r\| < \varepsilon$, stop converged.
2. Evaluate $J$ and $\lambda^2$, compute $\delta$. If the system is singular, stop unconverged.
3. If $\|\delta\| > \Delta$, rescale $\delta \leftarrow \delta\,\Delta/\|\delta\|$. This is trust-region-like: the direction is kept and only the length is limited.
4. Retract: $x \leftarrow x \oplus \delta$.

After $N$ iterations the residual is recomputed at the returned state, which is converged only if it is within tolerance. The reported residual always belongs to the returned state.

### Fixed Points and the Effect of Damping

The damped step vanishes exactly when $J^\top r = 0$. For a tall problem this is a stationary point of the cost. For a wide problem of full row rank it forces $r = 0$. Damping changes the path, not the destination: convergence drops from quadratic to linear, but a zero-residual fixed point stays a fixed point. A non-zero final residual therefore means an unreachable target, a singularity or an active bound.

## Complexity Analysis

| Item                    | Cost                                               | Justification                                                                                                                 |
|-------------------------|----------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| Normal matrix           | $O(R\,P\,m)$                                       | $J^\top J$ costs $R P^2$, $JJ^\top$ costs $P R^2$; $m = \min(R, P)$                                                           |
| Factorization and solve | $O(m^3)$                                           | LU of an $m \times m$ matrix dominates the $O(m^2)$ triangular solves                                                         |
| Whole solve             | at most $N$ iterations                             | each adds one residual and one Jacobian from the problem; $N + 1$ residual evaluations in all, the last at the returned state |
| Memory                  | one $R \times P$ Jacobian, one $m \times m$ system | stack only; no heap, no recursion                                                                                             |

## Step-by-Step Walkthrough

**Rosenbrock residuals.** $r = (10(y - x^2),\; 1 - x)$, $J = \begin{pmatrix} -20x & 10 \\ -1 & 0 \end{pmatrix}$, zero residual at $(1, 1)$. Start at $(-1.2, 1)$ with $\lambda = 0$; $R = P = 2$, so the step is solved in residual space.

| Iteration | State        | $r$                            | $J$                                                | Step $\delta$  | New state    |
|-----------|--------------|--------------------------------|----------------------------------------------------|----------------|--------------|
| 1         | $(-1.2, 1)$  | $(-4.4, 2.2)$, $\|r\| = 4.919$ | $\begin{pmatrix} 24 & 10 \\ -1 & 0 \end{pmatrix}$  | $(2.2, -4.84)$ | $(1, -3.84)$ |
| 2         | $(1, -3.84)$ | $(-48.4, 0)$, $\|r\| = 48.4$   | $\begin{pmatrix} -20 & 10 \\ -1 & 0 \end{pmatrix}$ | $(0, 4.84)$    | $(1, 1)$     |

The second residual is linear in $x$, so step 1 zeroes it exactly. The first is then linear in $y$, so step 2 finishes exactly. The residual norm rises from $4.9$ to $48.4$ on the way: the iteration is not monotone. With a step cap $\Delta = 1$, the first step of norm $5.316540$ is scaled to unit length, giving $(-0.786197,\; 0.089634)$.

**Angle on the unit circle.** State $\theta$, $r = (\cos\theta, \sin\theta) - (\cos 2.5, \sin 2.5)$, $J = (-\sin\theta,\; \cos\theta)^\top$. Here $R = 2 > P = 1$ and $J^\top J = 1$, so with $\lambda = 0$ each step is $\theta \leftarrow \theta - \sin(\theta - 2.5)$, wrapped into $(-\pi, \pi]$. Start at $\theta = -2.5$, which is only $1.28$ rad from the target across the $\pm\pi$ seam.

| Iteration | $\theta$    | $\sin(\theta - 2.5)$ | Before wrap | After wrap |
|-----------|-------------|----------------------|-------------|------------|
| 1         | $-2.500000$ | $0.958924$           | $-3.458924$ | $2.824261$ |
| 2         | $2.824261$  | $0.318608$           | $2.505653$  | $2.505653$ |
| 3         | $2.505653$  | $0.005653$           | $2.500000$  | $2.500000$ |

The wrap keeps the state in its canonical range, so the answer is $2.5$ and not $2.5 - 2\pi$. The error falls $0.324 \to 0.0057 \to 3\times10^{-8}$, faster than quadratic because of the circle's symmetry.

**Unreachable target.** Target $(2, 0)$ lies outside the unit circle, with $\lambda^2 = 1$. Then $J^\top J + \lambda^2 = 2$ and $\delta = -\tfrac12 J^\top r = -\sin\theta$, so $\theta \leftarrow \theta - \sin\theta \to 0$. At $\theta = 0$ the step vanishes ($J^\top r = 0$) while $\|r\| = \|(1, 0) - (2, 0)\| = 1$. The solve exhausts its budget and reports not converged.

## Pitfalls & Edge Cases

- **Squared condition number in single precision.** Forming $J^\top J$ or $JJ^\top$ squares the condition number of $J$, and the square case pays it too. On the Rosenbrock start $\operatorname{cond}(JJ^\top) \approx 4600$, and float32 Gauss–Newton reaches only $\|r\| \approx 10^{-5}$ after its two exact steps. Use tolerances of at least $10^{-4}$ in single precision.
- **Singular undamped system.** With $\lambda = 0$ and a rank-deficient $J$ the solve stops unconverged at the current state, with the iteration count reached so far. Use positive damping near singularities. A small positive $\lambda^2$ can pass the pivot test yet allow steps up to about $\|r\|/(2\lambda)$, so pair it with a step cap.
- **No cost-decrease check, local method.** The step is accepted unconditionally, so a poor initial guess can overshoot, as the Rosenbrock walkthrough shows, and with several minima the initial state decides which one is reached. A step cap and a warm start limit the risk; neither guarantees descent.
- **Non-zero minimum residual.** Convergence is tested only on $\|r\| < \varepsilon$. A problem whose best fit leaves a residual above $\varepsilon$ (noisy over-determined data, unreachable target) runs the full budget and reports not converged. The returned residual shows the value attained.
- **Jacobian and retraction must agree.** The Jacobian must be taken with respect to the very increment the retraction consumes. A mismatch, such as a left perturbation in the Jacobian and a right update, degrades convergence to linear or stalls it.
- **Active bounds.** A clamping retraction applies an unconstrained step and then projects it. Components pressed against a bound stop moving while the others continue, and the residual may stay non-zero.

## Variants & Generalizations

| Variant                                      | Key difference                                                                                                                                        |
|----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Gauss–Newton ($\lambda = 0$)**             | Quadratic convergence for zero-residual problems; fails on rank deficiency                                                                            |
| **Constant or problem-aware damping (this)** | Fixed $\lambda$ gives robust linear convergence; a Jacobian-dependent $\lambda^2$ (zero away from singularities) keeps the fast rate where safe       |
| **Gain-ratio (Nielsen) adaptation**          | Raises or lowers $\lambda$ from the ratio of actual to predicted cost reduction; needs a trial evaluation and accept/reject, a possible later variant |

## Applications

- **Inverse kinematics** — pose targets for manipulators, orientation on a manifold, joint limits through clamping, singularity-robust damping.
- **Parallel-robot forward kinematics** — platform pose from measured leg lengths.
- **Calibration and scan matching** — sensor models, hand-eye transforms and rigid alignment of measurements against a reference.
- **Cyclic control tasks** — warm start from the previous cycle's solution, which typically needs few iterations and gives a fixed worst-case cost.

## Connections to Other Algorithms

| Algorithm                                                  | Relationship                                                                                                                                                                |
|------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [LU Decomposition](../solvers/LuDecomposition.md)          | Solves the damped normal system; its singularity report ends the solve unconverged                                                                                          |
| [Damped Pseudo-Inverse](../solvers/DampedPseudoInverse.md) | The same damped step in SVD form for one linear solve; avoids squaring the condition number at higher cost                                                                  |
| [SE(3) Transform](../math/SE3Transform.md)                 | Supplies the manifold retraction for pose states                                                                                                                            |
| [Angle Utilities](../math/AngleUtilities.md)               | Supplies the wrapped retraction for angle states                                                                                                                            |
| [Gradient Descent](Optimizer.md)                           | First-order method for general costs; this method exploits the sum-of-squares structure through the Gauss–Newton curvature and needs far fewer iterations on small problems |

## References & Further Reading

- Levenberg, K., "A method for the solution of certain non-linear problems in least squares," *Quarterly of Applied Mathematics*, 2(2):164–168, 1944.
- Marquardt, D.W., "An algorithm for least-squares estimation of nonlinear parameters," *SIAM Journal on Applied Mathematics*, 11(2):431–441, 1963.
- Nocedal, J. and Wright, S.J., *Numerical Optimization*, 2nd ed., Springer, 2006 — Chapter 10.
- Absil, P.-A., Mahony, R., and Sepulchre, R., *Optimization Algorithms on Matrix Manifolds*, Princeton University Press, 2008 — Chapter 4 (retractions).
