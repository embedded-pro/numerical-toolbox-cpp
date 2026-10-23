# Active-Set Quadratic Programming (Goldfarb–Idnani)

## Overview & Motivation

Many embedded decisions are quadratic programs: distribute a wrench over cables that can only pull, allocate thrust to motors that saturate, track prioritised tasks under joint limits, keep model-predictive inputs inside bounds. Clipping the unconstrained answer, or running a fixed number of projection sweeps, is cheap but can be wrong at the edges, where several limits interact.

This solver finds the exact minimiser of a strictly convex quadratic cost under linear equality and inequality constraints, in a bounded number of steps and without dynamic memory. It is a **dual active-set** method: it starts at the unconstrained minimum, which violates some constraints, and repeatedly pulls the point towards the most violated one along the direction that raises the cost least. Every intermediate point is optimal for the constraints chosen so far, so no feasible starting point is needed, and a running orthogonal factorisation keeps each step at quadratic cost.

## Mathematical Theory

### Problem Statement

$$
\min_{x \in \mathbb{R}^n} \; \tfrac12 x^\top H x + g^\top x \quad \text{s.t.} \quad E x = e, \;\; C x \ge c,
$$

with $H$ symmetric positive definite. Strict convexity makes the minimiser unique whenever the feasible set is non-empty. Write the rows of $E$ followed by the rows of $C$ as normals $n_i$ with bounds $b_i$, $m = $ (number of equalities) $+$ (number of inequalities). A bound $\ell_k \le x_k \le h_k$ is two inequality rows: $x_k \ge \ell_k$ and $-x_k \ge -h_k$.

### Optimality Conditions

A point $x$ is optimal exactly when there are multipliers with

$$
H x + g = E^\top \lambda + C^\top \mu, \qquad \mu \ge 0, \qquad \mu_i\,(n_i^\top x - b_i) = 0,
$$

and $x$ is feasible. Equality multipliers $\lambda$ are free in sign; inequality multipliers $\mu$ are non-negative and zero for inactive rows. The algorithm maintains stationarity and $\mu \ge 0$ at every iteration and reaches feasibility only at the end: the cost of the iterate rises monotonically towards the optimum.

### Factored Form (Powell)

Factor $H = L L^\top$ and let $N$ collect the active normals. The change of variables $y = L^\top x$ turns the cost into $\tfrac12\|y\|^2 + (L^{-1}g)^\top y$ and each normal into $L^{-1} n_i$. A QR factorisation of these transformed normals gives the stable pair

$$
J = L^{-\top} Q, \qquad J^\top N = \begin{bmatrix} R \\ 0 \end{bmatrix}, \qquad J^\top H J = I,
$$

with $Q$ orthogonal and $R$ upper triangular, $q \times q$ for $q$ active constraints. Split $J = [J_1\; J_2]$ at column $q$. The columns of $J_2$ satisfy $N^\top J_2 = 0$: moving along them keeps every active constraint satisfied. Since $J J^\top = H^{-1}$, products with $J$ replace any explicit inverse of $H$, which is what keeps the update numerically stable.

### Adding a Constraint

Let $p$ be the constraint to add, $n^+$ its normal oriented so that the current residual $s = n^{+\top} x - b^+$ is negative. Define

$$
d = J^\top n^+ = \begin{bmatrix} d_1 \\ d_2 \end{bmatrix}, \qquad z = J_2 d_2, \qquad r = R^{-1} d_1 .
$$

Then $n^+ = N r + H z$ and $n^{+\top} z = \|d_2\|^2$. Moving $x \leftarrow x + t z$, $u \leftarrow u - t r$ on the active multipliers and $u_p \leftarrow u_p + t$ preserves stationarity, leaves the active constraints satisfied, and changes the residual of $p$ by $t\,\|d_2\|^2$. Two events can stop the step:

- **Full step** — the residual reaches zero: $t_2 = -s / (z^\top n^+)$, defined unless $n^+$ is linearly dependent on the active normals.
- **Partial step** — an active inequality multiplier reaches zero: $t_1 = \min\{u_j / r_j : r_j > 0\}$ over active inequalities.

With $t = \min(t_1, t_2)$ the multipliers and, if $t_2$ is finite, $x$ are updated. If $t = t_2$, constraint $p$ becomes active. If $t = t_1$, the blocking constraint is dropped and the step repeats for the same $p$ with a smaller active set. If $t = \infty$, nothing bounds the multiplier of $p$ and the constraints are contradictory: the problem is infeasible.

### Updating the Factors

- **Add:** Givens rotations on adjacent columns of $J_2$ zero all but the first entry of $d_2$; the rotated $d$ becomes the new last column of $R$, which grows by one.
- **Drop:** deleting column $k$ of $R$ leaves it upper Hessenberg; Givens rotations on adjacent rows restore triangularity and are applied to the same columns of $J$ to keep $J^\top N = [R;0]$.

### Linear Dependence Test

In exact arithmetic $n^+$ depends on the active normals iff $d_2 = 0$. In single precision $d_2$ is a tiny nonzero vector, and an absolute test on $\|z\|$ lets the step $-s/(z^\top n^+)$ divide by a rounding artefact. The shipped solver therefore uses a relative test:

$$
\|d_2\| \le 10^{-5}\,\|d\| .
$$

Since $J$ transforms by an orthogonal $Q$, the ratio $\|d_2\|/\|d\|$ is the sine of the angle between the transformed new normal and the span of the active ones. A dependent constraint takes no full step; only a dual step with $t_1$ remains, which is how degenerate active sets resolve.

### Termination

Equalities are added first, in order; an equality whose residual is positive is added with its normal and bound negated, and its multiplier is reported with the original sign. Equalities are never dropped. Then, while some inactive inequality has residual below $-\tau\,\|n_i\|$ (relative tolerance, default $\tau = 10^{-6}$), the one with the most negative raw residual is added. Each completed addition raises the cost, so in exact arithmetic no active set repeats; an outer cap (default $2m + n$) and a bound on drops per addition guard against rounding-induced cycling. Hitting either returns an iteration-limit status with the current finite $x$, which is dual feasible but may violate constraints.

## Complexity Analysis

With $n$ variables, $m$ constraints, $q \le n$ active, and $K$ the iteration cap:

| Case | Time | Space | Notes |
|------|------|-------|-------|
| Setup | $O(n^3)$ | $O(n^2)$ | Cholesky, triangular inverse $J = L^{-\top}$, start point $x = -J J^\top g$ |
| Best | $O(n^3 + mn)$ | $O(n^2 + m)$ | Unconstrained minimum already feasible: one residual scan |
| Average | $O(n^3 + k\,(n^2 + mn))$ | $O(n^2 + m)$ | $k$ additions, a few drops; per addition $d$, $z$, $r$ and rotations cost $O(n^2)$, the scan $O(mn)$ |
| Worst | $O(n^3 + K\,(n + m)\,n^2)$ | $O(n^2 + m)$ | Cap reached; loose bound, each addition allows at most $n + m + 1$ add/drop passes |

Storage is $J$ and $R$ ($n \times n$ each), multipliers, signs and an active-set list of length at most $n$; all on the stack, no heap.

## Step-by-Step Walkthrough

Minimise $\tfrac12\|x\|^2$ ($H = I$, $g = 0$) subject to $x_2 \ge 1$ and $0.1\,x_1 + 0.5\,x_2 \ge 0.9$. Start at $x = 0$ with $J = I$; residuals are $-1$ and $-0.9$, so the first row is the most violated.

| Step | Action | $t_1$ | $t_2$ | Result |
|------|--------|-------|-------|--------|
| 1 | Add row 1: $z = (0,1)$, no active set | $\infty$ | $1/1 = 1$ | $x = (0, 1)$, $u_1 = 1$, row 1 active |
| 2 | Add row 2: $n^+ = (0.1, 0.5)$; component along row 1 is $d_1 = 0.5$, orthogonal part $z = (0.1, 0)$, $r = 0.5$, residual $-0.4$ | $1/0.5 = 2$ | $0.4/0.01 = 40$ | Partial step $t = 2$: $x = (0.2, 1)$, $u_1 = 0$, $u_2 = 2$; row 1 dropped |
| 3 | Repeat row 2 with empty active set: $z = n^+$, residual $-0.38$ | $\infty$ | $0.38/0.26 = 1.4615$ | $x = (0.346154, 1.730769)$, $u_2 = 3.461538$ |

No inactive row is violated ($x_2 = 1.73 \ge 1$), so the solver stops: $x = 0.9\,(0.1, 0.5)/0.26$, the projection onto the second row, with multiplier $0.9/0.26 = 3.461538$ and row 1 inactive.

Other reference points:

- $\min \tfrac12\|x - (1,1)\|^2$ s.t. $x_1 + x_2 \le 1$ (row $-x_1 - x_2 \ge -1$): residual $-1$, $t_2 = 1/2$, so $x = (0.5, 0.5)$ and $\mu = 0.5$.
- $\min \tfrac12\|x\|^2$ s.t. $x_1 + x_2 = 1$: residual $-1$, $t_2 = 1/2$, so $x = (0.5, 0.5)$ and $\lambda = 0.5$.
- $\min \tfrac12\|x - (2,-1)\|^2$ over the box $[0,1]^2$: the violated bounds have orthogonal normals, so both are added with full steps and no drop, giving $x = (1, 0)$ with multipliers $1$ and $1$.

## Pitfalls & Edge Cases

- **Strict convexity is required.** A semidefinite or indefinite $H$ makes the Cholesky factorisation fail and the solver returns a not-positive-definite status. For an LP-like problem add $\varepsilon I$; since $J$ scales as $1/\sqrt{\varepsilon}$, pick $\varepsilon$ large enough to keep the conditioning benign in single precision.
- **Linear-dependence threshold (single precision).** The textbook absolute test on $\|z\|$ failed: a dependent constraint leaves a tiny nonzero $z$ and the solver takes a wild step. In a 3000-problem random test it reported 73 infeasible problems as optimal. The relative test $\|d_2\| \le 10^{-5}\|d\|$ matched a brute-force enumeration of KKT active sets on all 3000; it still does at $10^{-4}$ and fails at $10^{-3}$.
- **Selection by raw residual.** The constraint added is the one with the largest raw violation; only the acceptance test is relative to the row norm. Scale rows to comparable norms, otherwise a large-norm row hides the others.
- **Dependent equalities.** A consistent but linearly dependent equality has no blocking multiplier and is reported infeasible. Remove redundant equalities beforehand.
- **Contradictory constraints.** Detected when the step is unbounded; the returned point is not a solution.
- **Early stop.** After an iteration limit the point is dual feasible, so its cost is a lower bound on the optimum, but it may violate constraints.
- **Degenerate vertices.** More than $n$ rows binding at one point is handled by the drop branch; no more than $n$ constraints are ever active.
- **Feasibility tolerance.** Acceptance tolerates a residual down to $-\tau\|n_i\|$, so the returned point can violate a row by about that amount.
- **Measured accuracy.** On 3000 random problems of shapes (2 variables, 1 equality, 2 inequalities), (2, 0, 4) and (4, 2, 8) with duplicated, negated, dependent and badly scaled rows, the optimal point matched the oracle to about $10^{-4}$ relative, the scaled KKT residual was at most $4.4\times10^{-6}$, primal infeasibility at most $1.2\times10^{-5}$, and no inequality multiplier was negative.

## Variants & Generalizations

- **Primal active set:** keeps every iterate feasible and needs a Phase I to find a feasible start; Goldfarb–Idnani needs none but requires strictly convex $H$.
- **Warm start:** reuse the previous active set and factors when consecutive problems differ slightly, as in a receding-horizon loop; a possible extension, not provided.
- **Interior-point and first-order (splitting) methods:** they accept semidefinite $H$, but return approximate solutions and no exact active set.
- **Specialised bounds:** handling $\ell_k \le x_k \le h_k$ implicitly avoids stating each bound as two dense rows.
- **Projected coordinate descent for boxes:** cheaper per iteration, but a fixed sweep count gives only an approximate answer.

## Applications

- **Control allocation:** distribute a demanded force and torque over saturating actuators, minimising effort exactly.
- **Cable-tension distribution:** find tensions that realise a wrench while every cable only pulls.
- **Task-priority and whole-body control:** tracking objectives with joint and torque limits.
- **Model predictive control:** input bounds, and box or rate constraints on a condensed problem.
- **Constrained trajectory generation:** minimum-snap segments with position and velocity limits.
- **Least-distance projection:** nearest point of a polytope, as in the box example above.

## Connections to Other Algorithms

| Algorithm | Relationship |
|-----------|--------------|
| Cholesky decomposition | Supplies $L$ with $H = L L^\top$, and is the test for strict convexity |
| Triangular solve | Builds $J = L^{-\top}$ once and back-substitutes $R^{-1} d_1$ every pass |
| Givens rotation | Restores the triangular form of $R$ when a constraint is added or dropped |
| QR decomposition | $J$ and $R$ are an incrementally updated QR of the active normals in the $H^{-1}$ metric |
| Damped pseudo-inverse | The unconstrained counterpart: with no active constraint the answer is $-H^{-1} g$; this solver adds exact limits |
| Model predictive control | Its input bounds are a direct use of the bound rows; the current fixed-count projected descent could be replaced by this solver |

## References & Further Reading

- D. Goldfarb and A. Idnani, "A numerically stable dual method for solving strictly convex quadratic programs," *Mathematical Programming*, 27(1), 1983.
- M. J. D. Powell, "On the quadratic programming algorithm of Goldfarb and Idnani," *Mathematical Programming Study*, 25, 1985.
- J. Nocedal and S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006, Ch. 16.
