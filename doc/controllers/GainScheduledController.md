# Gain-Scheduled Controller

## Overview & Motivation

Many physical plants behave as linear systems only near a specific operating point. A motor's electrical dynamics change with rotational speed, an aircraft's aerodynamic response shifts with altitude and Mach number, and a chemical reactor's behaviour depends on temperature. Designing a single linear controller that performs well across the entire operating envelope is often impossible or overly conservative.

The gain-scheduled controller addresses this by pre-computing a family of linear controllers — each optimal near one operating point — and blending between them at runtime based on a measured **scheduling variable**. This extends the reach of proven linear design methods to mildly nonlinear plants at negligible runtime cost, making it the dominant approach to flight control, powertrain management, and process control on resource-constrained hardware.

## Mathematical Theory

### Setup

Let $\sigma \in \mathbb{R}$ be the scheduling variable (e.g., speed, load, altitude) and let $\mathbf{g}(\sigma) \in \mathbb{R}^p$ denote the vector of $p$ controller gains as a function of $\sigma$.

A finite set of $N$ **breakpoints** $\sigma_1 < \sigma_2 < \cdots < \sigma_N$ partitions the scheduling axis. At each breakpoint $\sigma_i$ a gain vector $\mathbf{g}_i \in \mathbb{R}^p$ is stored, pre-designed offline (e.g., by LQR at each linearised operating point).

### Piecewise-Linear Interpolation

For a scheduling value $\sigma$ in the interval $[\sigma_i, \sigma_{i+1}]$, the active gain vector is the linear interpolant:

$$
\mathbf{g}(\sigma) = \mathbf{g}_i + w \, (\mathbf{g}_{i+1} - \mathbf{g}_i), \qquad w = \frac{\sigma - \sigma_i}{\sigma_{i+1} - \sigma_i} \in [0,1].
$$

Each gain component is interpolated independently. The rearranged form $a + w(b - a)$ is numerically preferable to $(1-w)a + wb$ because it avoids catastrophic cancellation near $w \approx 0$ and performs one fewer multiply.

### Endpoint Saturation

Outside the table range the gains are held constant at the boundary values:

$$
\mathbf{g}(\sigma) = \begin{cases} \mathbf{g}_1 & \sigma \leq \sigma_1 \\ \mathbf{g}_N & \sigma \geq \sigma_N \end{cases}
$$

Extrapolation is avoided because gain values outside the design envelope are undefined and potentially destabilising.

### Stability Considerations

Gain scheduling does not guarantee closed-loop stability in general. The standard sufficient conditions are: (1) the scheduling variable changes slowly relative to the closed-loop bandwidth (quasi-static assumption), and (2) each frozen-$\sigma$ system is stable. When the scheduling variable changes rapidly, a full parameter-varying analysis (LPV, $\mathcal{H}_\infty$) is required.

## Complexity Analysis

| Case | Time | Space | Notes |
|------|------|-------|-------|
| Schedule (linear search) | $O(N + p)$ | $O(1)$ | $N$ interval scan, $p$ blends |
| Schedule (binary search) | $O(\log N + p)$ | $O(1)$ | Preferred for large tables |
| Construction | $O(N)$ | $O(N \cdot p)$ | Monotonicity assertion |

All storage is stack-allocated. The table occupies $N \cdot p$ words; the active gain vector occupies $p$ words. No dynamic allocation occurs at any point.

## Step-by-Step Walkthrough

**Table:** $\{(0, 1), (0.5, 2), (1, 4)\}$ — three breakpoints, one gain each.

**Query $\sigma = 0.75$:**

1. $\sigma = 0.75 > \sigma_1 = 0$ and $\sigma = 0.75 < \sigma_3 = 1.0$, so no saturation.
2. Locate interval: $\sigma_2 = 0.5 \leq 0.75 < \sigma_3 = 1.0$, so $i = 2$.
3. Blend weight: $w = (0.75 - 0.5) / (1.0 - 0.5) = 0.5$.
4. Active gain: $g = 2.0 + 0.5 \times (4.0 - 2.0) = 3.0$.

**Query $\sigma = -1.0$** (below table): saturate to $g = g_1 = 1.0$.

## Pitfalls & Edge Cases

- **Non-monotone breakpoints.** The interpolation formula divides by $\sigma_{i+1} - \sigma_i$; equal or reversed breakpoints cause division by zero or sign inversion. Strictly increasing order must be asserted at construction.
- **Rapid scheduling.** If $\dot{\sigma}$ is large relative to the closed-loop bandwidth, the quasi-static stability argument breaks down. The scheduling variable should be filtered or the breakpoint density increased near fast-varying regions.
- **Extrapolation is unsafe.** Outside the design envelope, gains are unknown quantities. Endpoint saturation is the only safe embedded policy; never extrapolate control gains.
- **Gain discontinuities.** If adjacent breakpoint gain sets differ sharply, the interpolated gains may produce a transient step in the control output when $\sigma$ crosses a breakpoint. Smooth scheduling designs (e.g., LPV synthesis) eliminate this.
- **Single-breakpoint interval.** Degenerate tables with $N = 1$ offer no interpolation; require $N \geq 2$ at compile time.

## Variants & Generalizations

| Variant | Key Difference |
|---------|---------------|
| **Bilinear / 2-D scheduling** | Gains indexed by two variables (e.g., speed and load); uses bilinear interpolation on a grid |
| **LPV (Linear Parameter-Varying)** | Gains are polynomial functions of $\sigma$; stability guaranteed by parameter-dependent Lyapunov functions |
| **Velocity-form scheduling** | Gains on incremental (velocity-form) controllers avoid output steps at scheduling transitions |
| **Bumpless transfer** | State initialisation at switchover prevents transients when $\sigma$ jumps discontinuously |

## Applications

- **Flight control** — gain sets designed at different Mach / altitude corners; $\sigma =$ dynamic pressure or Mach number.
- **Automotive powertrain** — PID gains scheduled on engine speed and load to compensate torque-speed nonlinearity.
- **Industrial process control** — reactor temperature or pH controllers scheduled on throughput or feed concentration.
- **Servo drives** — current/velocity loop gains scheduled on rotor speed to compensate back-EMF and inductance variation.

## Connections to Other Algorithms

| Algorithm | Relationship |
|-----------|-------------|
| [LQR](Lqr.md) | Common source of the per-breakpoint gain sets; each $\mathbf{g}_i$ is an LQR solution at the $i$-th linearised operating point |
| [Feedforward/2-DOF](Feedforward2Dof.md) | Complementary structure: the feedforward path can also be gain-scheduled to account for reference-dependent plant changes |
| [MPC](Mpc.md) | Alternative that solves an optimisation online; eliminates the need for offline scheduling but requires more computation |

## References & Further Reading

- W. J. Rugh, J. S. Shamma, "Research on gain scheduling," *Automatica*, 36(10), 1401–1425, 2000.
- J. S. Shamma, M. Athans, "Gain scheduling: Potential hazards and possible remedies," *IEEE Control Systems Magazine*, 12(3), 101–107, 1992.
- D. J. Leith, W. E. Leithead, "Survey of gain-scheduling analysis and design," *International Journal of Control*, 73(11), 1001–1025, 2000.
