# Cubic Spline Interpolation (Vector-Valued, Clamped / Natural)

## Overview & Motivation

Waypoint lists (robot teach-in programs, CNC paths, planner output), calibration tables and resampled signals all need a smooth curve through given samples. A single high-order polynomial through many points rings between them; a piecewise-linear curve has corners that excite mechanics. A **cubic spline** joins one cubic per interval so that value, slope **and** curvature are continuous ($C^2$), and it stays well behaved however many knots are used.

Several channels (the joints of a robot, the axes of a path, the columns of a table) share the same abscissae. They are fitted together: the linear system depends only on the knot spacing, so it is factored once and every channel reuses the factors. Fitting is a single linear-time pass on fixed storage; evaluation is a binary search plus a cubic, cheap and deterministic.

## Mathematical Theory

### Unknowns and per-interval form

Knots $x_0 < x_1 < \dots < x_{K-1}$ with spans $h_k = x_{k+1} - x_k$ and values $y_k$ (one component per channel; channels are independent). The unknowns are the knot second derivatives $M_k = y''(x_k)$. On interval $k$, with $\tau = x - x_k \in [0, h_k]$,

$$y(\tau) = y_k + \beta_k\,\tau + \tfrac{M_k}{2}\,\tau^2 + d_k\,\tau^3, \qquad d_k = \frac{M_{k+1} - M_k}{6\,h_k}, \qquad \beta_k = \frac{y_{k+1} - y_k}{h_k} - \frac{h_k\,(2M_k + M_{k+1})}{6}$$

The cubic hits both knot values and both knot curvatures, so value is $C^0$ and curvature is $C^2$ by construction. Only slope continuity remains to be imposed.

### Interior rows

Equating the slope at the end of interval $k-1$, $\beta_{k-1} + h_{k-1}(M_{k-1} + M_k)/2$, with $\beta_k$ and writing $s_k = (y_{k+1} - y_k)/h_k$ gives, for $k = 1, \dots, K-2$,

$$h_{k-1} M_{k-1} + 2(h_{k-1} + h_k)\,M_k + h_k M_{k+1} = 6\,(s_k - s_{k-1})$$

Each row couples a knot to its two neighbours only, so the system is tridiagonal with $K-2$ rows. Two end conditions supply the missing two.

### End conditions

| Condition                                       | First row                                  | Last row                                                       |
|-------------------------------------------------|--------------------------------------------|----------------------------------------------------------------|
| **Clamped** (end slopes $\sigma_0$, $\sigma_1$) | $2h_0 M_0 + h_0 M_1 = 6\,(s_0 - \sigma_0)$ | $h_{K-2} M_{K-2} + 2h_{K-2} M_{K-1} = 6\,(\sigma_1 - s_{K-2})$ |
| **Natural**                                     | $M_0 = 0$                                  | $M_{K-1} = 0$                                                  |

Clamping fixes the end slopes and leaves the end second derivatives free; natural ends do the reverse. Default slopes are zero.

### Solvability

Every interior row has diagonal $2(h_{k-1} + h_k)$ against off-diagonal sum $h_{k-1} + h_k$; clamped end rows have $2h$ against $h$; natural rows have $1$ against $0$. All rows are strictly diagonally dominant, so the matrix is nonsingular and the [Thomas algorithm](TridiagonalSolver.md) is stable without pivoting. The matrix contains only spans and the end-condition kind, never values or slopes: one factorization serves all channels, and only the right-hand sides differ.

### Evaluation

Clamp $x$ to $[x_0, x_{K-1}]$, find the interval by binary search, then use Horner's scheme per channel:

$$y = y_k + \tau\,\bigl(\beta_k + \tau\,(\tfrac{M_k}{2} + \tau\,d_k)\bigr), \quad y' = \beta_k + \tau\,(M_k + 3 d_k \tau), \quad y'' = M_k + 6 d_k \tau$$

An abscissa exactly on an interior knot selects the interval starting there; the last knot selects the final interval. Both choices give the same result because the spline is $C^2$.

## Complexity Analysis

| Operation     | Time            | Space                      | Notes                                                                              |
|---------------|-----------------|----------------------------|------------------------------------------------------------------------------------|
| Factorization | $O(K)$          | three bands                | Once per fit, shared by all channels                                               |
| Fit           | $O(D\,K)$       | $K_{\max}(2 + 2D)$ scalars | $D$ channels, each one right-hand side build and one solve of $O(K)$; no iteration |
| Evaluate      | $O(\log K + D)$ | $O(1)$                     | Binary search, then three Horner polynomials per channel                           |

Storage holds the knots, their reciprocal spans, the values and the second derivatives: $K_{\max}(2 + 2D)$ scalars, where $K_{\max}$ is the compile-time capacity, plus the three bands of the solver. All of it is bounded and on the stack or inside the object; no heap.

## Step-by-Step Walkthrough

Knots $(0, 1, 2)$, values $(0, 1, 0)$, so $h_0 = h_1 = 1$ and $s_0 = 1$, $s_1 = -1$.

**Clamped, zero end slopes.** The system is
$$\begin{pmatrix} 2 & 1 & 0 \\ 1 & 4 & 1 \\ 0 & 1 & 2 \end{pmatrix} M = \begin{pmatrix} 6 \\ -12 \\ 6 \end{pmatrix}$$

| $k$ | Pivot $p_k$      | $c'_k$ | Forward $y_k$ | Back $M_k$          |
|-----|------------------|--------|---------------|---------------------|
| 0   | $2$              | $1/2$  | $3$           | $3 + 3 = 6$         |
| 1   | $4 - 1/2 = 7/2$  | $2/7$  | $-30/7$       | $-30/7 - 12/7 = -6$ |
| 2   | $2 - 2/7 = 12/7$ | unused | $6$           | $6$                 |

So $M = (6, -6, 6)$. Interval 0 has $\beta_0 = 1 - (12 - 6)/6 = 0$ and $d_0 = -2$, giving $y(\tau) = 3\tau^2 - 2\tau^3$. Hence $y(0.5) = 0.75 - 0.25 = 0.5$, $y'(0.5) = 3 - 1.5 = 1.5$ and $y''(0.5) = 0$. The curve leaves and enters every knot flat, like a smooth step.

**Natural.** The end rows reduce to $M_0 = M_2 = 0$, so the middle row is $4M_1 = -12$ and $M = (0, -3, 0)$. Interval 0 has $\beta_0 = 1 + 3/6 = 1.5$ and $d_0 = -0.5$, giving $y(\tau) = 1.5\tau - 0.5\tau^3$. Hence $y(0.5) = 0.75 - 0.0625 = 0.6875$ and $y'(0.5) = 1.125$. The end slope $y'(0) = 1.5$ is free, and the curvature is zero there.

### Verification

Random fits are checked for interpolation of every knot, for continuity of first and second derivative across each interior knot, and for the end conditions (prescribed slopes when clamped, zero curvature when natural). Continuity holds to about $10^{-13}$ in double precision and to the single-precision rounding level in float.

## Pitfalls & Edge Cases

- **Invalid knots.** Fewer than 2 knots, more than the capacity, or spacing $x_{k+1} - x_k \le \varepsilon \max(1, |x_k|)$ is rejected, where $\varepsilon$ is the machine epsilon of the scalar type. Near-coincident knots would make slope quotients and the second derivatives huge.
- **Failure leaves no fit.** A rejected or unfactorable fit discards any earlier fit; evaluating before a successful fit is a precondition violation.
- **Tolerance is a floor, not a guarantee.** Spacing only a few $\varepsilon$ wide passes the test yet produces enormous slopes; keep sample spacing meaningful relative to the data.
- **Overshoot.** A cubic spline interpolates; it does not bound the curve between knots, so limits (velocity, position) must be checked on the evaluated curve.
- **Outside the domain.** The abscissa is clamped, so the output is the end-knot sample, including its slope and curvature, which need not be zero. Position holds while the reported velocity does not drop to zero.
- **Pick the end condition per application.** Clamped when end velocities matter (start and stop at rest); natural when they do not, accepting free end slopes.
- **Degenerate cases.** Two clamped knots give exactly the cubic Hermite polynomial. Two natural knots give the straight line. Collinear data with a matching clamped slope give a zero right-hand side and so $M \equiv 0$: the line is reproduced exactly.
- **Floating-point only.** The reciprocal spans and the epsilon test have no meaning for integer types.

## Variants & Generalizations

- **Not-a-knot / periodic ends.** Other end rows keep the tridiagonal shape (not-a-knot) or add corner entries solved by Sherman–Morrison (periodic).
- **Shape-preserving cubics.** Monotone Hermite interpolation chooses slopes locally; it removes overshoot but gives up $C^2$ for $C^1$.
- **B-spline form.** Same curve family with local basis functions; suited to non-interpolating fits and variable degree.
- **Smoothing and least-squares splines.** Trade exact interpolation for noise rejection; see polynomial fitting for the non-interpolating alternative.

## Applications

- **Joint-space and Cartesian trajectories** through waypoints, with continuous velocity and acceleration.
- **CNC and planner paths** parameterized by time or arc length.
- **Calibration tables and resampling** of sensor signals onto a new grid.

## Connections to Other Algorithms

| Algorithm                                                     | Relationship                                                                                                             |
|---------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------|
| [Tridiagonal Solver](TridiagonalSolver.md)                    | Solves the second-derivative system; its factorization is reused across channels                                         |
| [Polynomial Profile](../motion_profiles/PolynomialProfile.md) | The two-knot clamped case is the same cubic Hermite polynomial; the spline generalizes it to many knots with $C^2$ joins |
| Polynomial fitting                                            | Least-squares fit instead of interpolation                                                                               |

## References & Further Reading

- de Boor, C., *A Practical Guide to Splines*, Springer, 1978 (rev. ed. 2001).
- Burden, R. L. & Faires, J. D., *Numerical Analysis*, §3.5 (cubic spline interpolation).
