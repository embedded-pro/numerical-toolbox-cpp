# H∞ State-Feedback Control

## Overview & Motivation

Robust control guarantees performance under worst-case conditions rather than average ones. H∞
state-feedback synthesizes a linear gain that minimizes the largest possible ratio of output energy
to disturbance energy across all bounded disturbances — the induced $\mathcal{L}_2$ gain. This
provable attenuation bound is what safety-critical and certification-driven embedded systems require:
not a best-guess response, but a hard upper limit on how badly an unknown disturbance can degrade
performance.

## Mathematical Theory

### Generalized Plant

The design is framed around the generalized plant

$$
x_{k+1} = A x_k + B_1 w_k + B_2 u_k, \quad z_k = C_1 x_k + D_{12} u_k
$$

where $w \in \mathbb{R}^{n_w}$ is the exogenous disturbance, $u \in \mathbb{R}^{n_u}$ is the
control input, and $z \in \mathbb{R}^{n_z}$ is the performance (error) output.

### H∞ Performance Criterion

The controller objective is to find $u = -K x$ such that

$$
\sup_{w \neq 0} \frac{\|z\|_2}{\|w\|_2} < \gamma
$$

where $\gamma > 0$ is the prescribed attenuation level. Finding the smallest feasible $\gamma$
(the optimal $\gamma^*$) determines the best achievable robustness.

### Game-Theoretic Discrete Algebraic Riccati Equation (GARE)

The H∞ state-feedback gain is derived from the discrete Riccati equation

$$
X = A^\top X A - A^\top X B \tilde{R}^{-1} B^\top X A + Q
$$

with the augmented input matrix $B = [B_2 \mid B_1]$ and the **indefinite** weight matrix

$$
\tilde{R} = \begin{pmatrix} I_{n_u} & 0 \\ 0 & -\gamma^2 I_{n_w} \end{pmatrix}.
$$

The negative block encodes the adversarial role of the disturbance: the disturbance player
maximizes while the control player minimizes. A positive-semidefinite stabilizing solution $X \geq 0$
exists if and only if $\gamma$ is above the optimal level $\gamma^*$.

### Gain Extraction

From the Riccati solution $X$, the augmented gain is

$$
K_{\text{full}} = (\tilde{R} + B^\top X B)^{-1} B^\top X A.
$$

Only the top $n_u$ rows — the control block — form the feedback gain $K$, and the closed-loop
map is $A_{\text{cl}} = A - B_2 K$.

### Feasibility Conditions

A given $\gamma$ is feasible when:
1. The GARE has a positive-semidefinite solution $X \geq 0$.
2. The disturbance block $-\gamma^2 I + B_1^\top X B_1 \prec 0$ is negative definite, confirming
   that the disturbance remains a genuine maximizer rather than a destabilizing force.

### Bisection for Optimal $\gamma$

Neither condition holds for $\gamma < \gamma^*$; both hold for $\gamma > \gamma^*$. A standard
bisection on $[\gamma_{\min}, \gamma_{\max}]$ converges to $\gamma^*$ at a linear rate.

## Complexity Analysis

| Phase          | Time Complexity                                             | Space    | Notes                              |
|----------------|-------------------------------------------------------------|----------|------------------------------------|
| Synthesize     | $O(\log((\gamma_{\max}-\gamma_{\min})/\epsilon) \cdot n^3)$ | $O(n^2)$ | Dominated by iterative DARE solves |
| ComputeControl | $O(n_u \cdot n)$                                            | $O(1)$   | Single matrix-vector multiply      |

## Step-by-Step Walkthrough

Consider a 2-state discrete plant with one disturbance and one control input.

1. **Bisect**: choose $g = (\gamma_{\min} + \gamma_{\max}) / 2$; stack $B = [B_2 \mid B_1]$;
   build $\tilde{R} = \text{diag}(1, -g^2)$.
2. **Solve GARE**: run the iterative DARE solver with indefinite $\tilde{R}$ until convergence.
3. **Check feasibility**: verify $X_{ii} \geq 0$ for all $i$ and $-g^2 + (B_1^\top X B_1)_{ii} < 0$.
4. **Update bisection**: if feasible, tighten ($\gamma_{\max} \leftarrow g$); otherwise relax
   ($\gamma_{\min} \leftarrow g$).
5. **Finalize**: at convergence, solve GARE at $\gamma_{\max}$, extract the control rows of
   $K_{\text{full}}$, and verify $A - B_2 K$ is Schur-stable (all eigenvalues inside the unit disk).

## Pitfalls & Edge Cases

- **Ill-conditioned GARE near $\gamma^*$**: the Riccati solution blows up as $\gamma \to \gamma^*$
  from above. The bisection tolerance should not be driven below the float precision of the
  Riccati solver.
- **Indefinite $\tilde{R}$**: the standard DARE assumes positive-definite $R$; using $\tilde{R}$
  with a negative block is valid only when the full augmented pair $(A, B)$ is stabilizable and
  the game saddle-point condition holds. Infeasibility manifests as non-PSD $X$ or violated
  disturbance-block condition.
- **Disturbance block check**: a numerically PSD $X$ does not guarantee feasibility; the
  disturbance block condition must also be verified explicitly.
- **Float precision**: accumulated rounding in many DARE iterations can erode the convergence
  criterion; using the iterative formulation with a conservative tolerance (relative to `1e-3f`)
  prevents premature acceptance of a diverged iterate.

## Variants & Generalizations

- **H∞ output feedback (H∞ LQG)**: replaces the state $x$ with an observer-based estimate; requires
  a second (filter) Riccati equation to solve the full information-state problem.
- **Continuous-time H∞**: replaces the DARE with the continuous algebraic Riccati equation;
  directly applicable to analog plants or zero-order-hold designs.
- **Mixed H₂/H∞**: constrains the H∞ norm while minimizing the H₂ (LQG) cost — trades average
  and worst-case performance on a Pareto frontier.

## Applications

- Safety-critical motion control where actuator saturation or load shifts make average-case design
  insufficient.
- Vibration suppression under unknown broadband disturbances.
- Robust attitude control of spacecraft or UAVs subject to unmodeled flexible modes.
- Robust stabilization of plants with parametric uncertainty encoded as bounded disturbances.

## Connections to Other Algorithms

- **LQR** ($\gamma \to \infty$): as the adversary weakens, the H∞ gain converges to the LQR gain
  for the same $(A, B_2, Q, I)$ weights. H∞ is the robust generalization of LQR.
- **DiscreteAlgebraicRiccatiEquation**: the inner computational engine; H∞ passes an indefinite
  weight to it.
- **LQG / Kalman Filter**: the stochastic average-case counterpart; H∞ and LQG bound opposite ends
  of the robustness-optimality trade-off.
- **Sliding Mode Control**: a nonlinear alternative to H∞ that achieves robust disturbance rejection
  without solving a Riccati equation, at the cost of chattering and switching nonlinearity.
- **DurandKerner**: used post-synthesis to verify that all eigenvalues of $A - B_2 K$ lie inside
  the unit disk.

## References & Further Reading

- J. Doyle, K. Glover, P. Khargonekar, B. Francis, "State-Space Solutions to Standard H₂ and H∞
  Control Problems," *IEEE Trans. Automatic Control*, 34(8), pp. 831–847, 1989.
- B. A. Francis, *A Course in H∞ Control Theory*, Lecture Notes in Control and Information
  Sciences, Springer, 1987.
- K. Zhou, J. C. Doyle, K. Glover, *Robust and Optimal Control*, Prentice-Hall, 1996.
