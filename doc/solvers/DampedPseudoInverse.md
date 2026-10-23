# Damped Pseudo-Inverse and Null-Space Projector

## Overview & Motivation

Mapping desired outputs to inputs through a non-square or ill-conditioned matrix $A$ is everywhere: joint rates from a tool velocity, motor commands from a desired thrust and torque, calibration corrections from residuals. The plain pseudo-inverse divides by every singular value, so near rank loss it explodes and commands huge, saturating inputs. **Damping** replaces the division by a gain that equals $1/\sigma$ for strong directions and fades to zero for weak ones, trading a small, bounded accuracy loss for a bounded answer.

When there are more inputs than outputs, $A$ has a null space: input motions it cannot feel. A **null-space projector** removes from any vector every component the matrix sees, so a secondary goal (stay away from limits, share load, save energy) can use the spare freedom without disturbing the primary one. Both tools come from one thin SVD, factored once per matrix.

## Mathematical Theory

### Factorization and Rank

For $A \in \mathbb{R}^{m \times n}$, tall or wide, and $k = \min(m, n)$, the thin SVD is $A = U\Sigma V^\top$ with $\sigma_0 \geq \cdots \geq \sigma_{k-1} \geq 0$, left vectors $u_i \in \mathbb{R}^m$ and right vectors $v_i \in \mathbb{R}^n$. With a relative tolerance $\tau$ (default $10^{-4}$) the rank threshold and rank are

$$\theta = \tau\,\sigma_0, \qquad r = \#\{\,i : \sigma_i > \theta\,\}.$$

Since the values are sorted, the counted directions are the leading $r$. The zero matrix has $\sigma_0 = 0$, hence $r = 0$.

### Damped Gain

Damped least squares (Tikhonov regularization) minimizes $\|Ax - b\|^2 + \lambda^2\|x\|^2$. Its normal equations give

$$x = (A^\top A + \lambda^2 I)^{-1}A^\top b = A^\top (AA^\top + \lambda^2 I)^{-1} b,$$

and in the singular basis each direction is scaled by the gain

$$g(\sigma) = \begin{cases} \dfrac{\sigma}{\sigma^2 + \lambda^2} & \lambda > 0, \\[2mm] 1/\sigma & \lambda = 0,\ \sigma > \theta, \\[1mm] 0 & \lambda = 0,\ \sigma \leq \theta. \end{cases}$$

For $\sigma \gg \lambda$ the gain is $\approx 1/\sigma$; for $\sigma \ll \lambda$ it is $\approx \sigma/\lambda^2 \to 0$. Its maximum is at $\sigma = \lambda$, so $g \leq 1/(2\lambda)$ always. Without damping the threshold zeroes weak directions, so no division by zero occurs. The solution and the damped inverse are

$$x = \sum_{i=0}^{k-1} g(\sigma_i)\,(u_i^\top b)\,v_i, \qquad A^\dagger_\lambda = \sum_{i=0}^{k-1} g(\sigma_i)\,v_i u_i^\top.$$

Every term lies in the span of the $v_i$, the row space of $A$, so $x$ has no null-space component: for $\lambda = 0$ it is the minimum-norm solution.

### Null-Space Projector

$$P = I - \sum_{i<r} v_i v_i^\top, \qquad Px = x - \sum_{i<r}(v_i^\top x)\,v_i.$$

The second form applies the projection without forming $P$: $r$ inner products and $r$ vector updates. The projector is deliberately undamped. Built from the damped inverse, $I - A^\dagger_\lambda A$ leaves a residual in the primary task:

$$A\,(I - A^\dagger_\lambda A) = \sum_i \sigma_i \frac{\lambda^2}{\sigma_i^2 + \lambda^2}\,u_i v_i^\top = \lambda^2 (AA^\top + \lambda^2 I)^{-1} A \neq 0.$$

Along each singular direction its eigenvalue is $\lambda^2/(\sigma_i^2 + \lambda^2)$, strictly between $0$ and $1$ for $\sigma_i > 0$, so it is not idempotent either. The thresholded projector is exact: $P^2 = P$, $P^\top = P$, and

$$AP = \sum_{i \geq r} \sigma_i\,u_i v_i^\top, \qquad \sigma_i \leq \theta,$$

which is zero for exact rank loss and at rounding level for full rank.

### Damping Bias

For $b \in \operatorname{range}(A)$ the residual is $b - Ax = \sum_i \frac{\lambda^2}{\sigma_i^2 + \lambda^2}(u_i^\top b)\,u_i$, hence

$$\|Ax - b\| \leq \frac{\lambda^2}{\sigma_{\min}^2 + \lambda^2}\,\|b\| \leq \frac{\lambda^2}{\sigma_{\min}^2}\,\|b\|,$$

with $\sigma_{\min}$ the smallest nonzero singular value. Robustness costs accuracy of order $\lambda^2/\sigma_{\min}^2$. Near rank loss the threshold grows the null space: the lost direction is released to the projector by design, while the damped solution stays bounded.

## Complexity Analysis

| Operation                   | Time                 | Space             | Notes                                                                    |
|-----------------------------|----------------------|-------------------|--------------------------------------------------------------------------|
| Factor                      | $O(Mk^2)$            | $O(mk + nk)$      | One Golub–Kahan SVD; wide matrices through their transpose               |
| Solve                       | $O((m + n)\,k)$      | $O(n)$            | $k$ inner products of length $m$, $k$ updates of length $n$; no division |
| Project                     | $O(n\,r)$            | $O(n)$            | $r$ inner products and updates; $P$ never formed                         |
| Explicit inverse, projector | $O(mnk)$, $O(n^2 r)$ | $O(mn)$, $O(n^2)$ | Diagnostics; sums of rank-one terms                                      |

Here $M = \max(m, n)$. The SVD factors are the only persistent storage: $U$ is $m \times k$, $V$ is $n \times k$, plus $k$ singular values. No heap is used, and the factorization is paid once per matrix; later solves and projections are inner products only.

## Step-by-Step Walkthrough

**Input:** $A = \begin{pmatrix} 1 & 0 & 1 \\ 0 & 1 & 1 \end{pmatrix}$, $b = (1, 2)$, $\tau = 10^{-4}$.

**Factor.** $AA^\top = \begin{pmatrix} 2 & 1 \\ 1 & 2 \end{pmatrix}$ has eigenvalues $3$ and $1$, so $\sigma = (\sqrt{3}, 1)$. Then $u_0 = (1, 1)/\sqrt{2}$, $u_1 = (1, -1)/\sqrt{2}$, $v_0 = (1, 1, 2)/\sqrt{6}$, $v_1 = (1, -1, 0)/\sqrt{2}$ (signs are arbitrary). The threshold is $\theta \approx 1.7 \times 10^{-4}$, so $r = 2$; the null space is spanned by $(1, 1, -1)$.

**Gains and projections** at $\lambda = 0.1$:

| $i$ | $\sigma_i$ | $u_i^\top b$ | $1/\sigma_i$ | $g(\sigma_i)$ |
|-----|------------|--------------|--------------|---------------|
| 0   | $1.732051$ | $2.121320$   | $0.577350$   | $0.575432$    |
| 1   | $1$        | $-0.707107$  | $1$          | $0.990099$    |

**Solution.** Without damping, $x = A^\dagger b = (0, 1, 1)$: it satisfies $Ax = b$ and is orthogonal to the null space ($0 + 1 - 1 = 0$). With $\lambda = 0.1$, $x = (0.003289, 0.993388, 0.996678)$. The residual is $b - Ax = \lambda^2(AA^\top + \lambda^2 I)^{-1}b$, of norm $0.009934$, within the bound $0.022139$ (the looser $\lambda^2\|b\|/\sigma_{\min}^2$ gives $0.022361$).

**Projector.** With both singular directions counted,

$$P = I - v_0 v_0^\top - v_1 v_1^\top = \tfrac{1}{3}\begin{pmatrix} 1 & 1 & -1 \\ 1 & 1 & -1 \\ -1 & -1 & 1 \end{pmatrix}, \qquad \operatorname{tr} P = 1 = n - r.$$

Projecting $(1, 0, 0)$ gives $(1/3, 1/3, -1/3)$, which $A$ maps to $0$. Projecting the damped solution gives $\approx 0$, since it lies in the row space.

**Rank loss and check.** For $A = \begin{pmatrix} 1 & 1 & 0 \\ 2 & 2 & 0 \end{pmatrix}$ the singular values are $(\sqrt{10}, 0)$: $r = 1$ and $\operatorname{tr} P = 2$. Undamped, the zero direction gets zero gain, and $A^\dagger(1, 2) = (0.5, 0.5, 0)$. The results were compared with the closed forms $(A^\top A + \lambda^2 I)^{-1}A^\top b$ (tall) and $A^\top (AA^\top + \lambda^2 I)^{-1} b$ (wide), evaluated in double precision, on random $2 \times 3$, $6 \times 7$, $7 \times 6$ and $4 \times 4$ matrices: solutions agree to about $10^{-5}$ and $\|P^2 - P\|$ is about $10^{-6}$.

## Pitfalls & Edge Cases

- **Choice of $\lambda$:** too small leaves large gains (up to $1/(2\lambda)$) near singularities; too large biases every direction by about $\lambda^2/\sigma^2$. Choose it from the smallest singular value that must still be tracked.
- **Hard rank decision:** the rank and the projector jump when a singular value crosses $\theta$, so secondary motion can switch abruptly near rank loss; smoothing or hysteresis belongs to the caller. The damped solution also keeps a small component $\sigma/(\sigma^2 + \lambda^2)\,(u^\top b)$ along directions already released to the projector, which the projection does not remove.
- **Degenerate shapes:** a zero matrix gives rank $0$, a zero solution and a projection equal to the identity; undamped, a zero singular value has zero gain, never a division. A tall full-rank matrix has a zero projector and an undamped least-squares solution.
- **Precision and failure:** $\tau$ must exceed the rounding level of $\sigma_0$ (about $10^{-7}$ in single precision), or noise is counted as rank. If the SVD does not converge the call reports failure and the results must not be used; $\lambda$ and $\tau$ must be non-negative. Singular-vector signs are arbitrary, and the solution and projector are invariant to them.

## Variants & Generalizations

- **Truncated SVD ($\lambda = 0$):** hard cutoff at $\theta$; exact on retained directions, discontinuous in $A$.
- **Adaptive damping:** $\lambda$ zero away from singularity, rising as $\sigma_{\min}$ falls below a margin (Nakamura and Hanafusa), so accuracy is lost only when needed.
- **Weighted damped least squares:** $\min \|W(Ax - b)\|^2 + \lambda^2\|Mx\|^2$ with task and joint weights, reduced to the plain form by scaling.

## Applications

- **Redundant manipulators and damped IK:** joint rates $\dot q = J^\dagger_\lambda \dot x + P z$ track the task velocity and use the null space for a secondary objective (joint-limit avoidance, posture) through gradient projection.
- **Control allocation:** distribute a commanded thrust and torque over more actuators than axes, with the projector holding the spare freedom for load sharing or energy minimization.
- **Regularized calibration:** correction estimates from residuals when the sensitivity matrix is ill-conditioned.

## Connections to Other Algorithms

- **Singular Value Decomposition** is the factorization; its plain thresholded pseudo-inverse is the undamped special case, and wide matrices are handled there through the transpose.
- **Condition Number** ($\sigma_0/\sigma_{\min}$) indicates when damping is needed and how to size $\lambda$.
- **QR Decomposition, LU Decomposition, Gaussian Elimination** solve full-rank least-squares or square systems more cheaply but reveal no rank and cannot damp gracefully.
- **Regularization and Levenberg–Marquardt:** the $\lambda^2\|x\|^2$ penalty is ridge regression; Levenberg–Marquardt applies the same damping per iteration to keep a Gauss–Newton step bounded.

## References & Further Reading

- C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods," *IEEE Trans. Systems, Man, and Cybernetics*, 16(1), 1986.
- Y. Nakamura and H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control," *J. Dynamic Systems, Measurement, and Control*, 108, 1986.
- G. H. Golub and C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins UP, 2013, §5.5.
