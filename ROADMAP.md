# Proposed Components Roadmap

Prioritized backlog of reusable numerical components for generic embedded applications,
ordered **easiest → hardest to implement** within the constraints of this library
(templated on `float` / `math::Q15` / `math::Q31`, no heap, bounded containers,
`OPTIMIZE_FOR_SPEED` hot paths, typed tests, `doc/` page).

Difficulty legend:

| Stars | Meaning                                                             | Typical effort |
|-------|---------------------------------------------------------------------|----------------|
| ★☆☆☆☆ | Trivial primitive — a few methods, minimal math                     | Hours          |
| ★★☆☆☆ | Easy — well-defined recurrence, small state                         | Half day       |
| ★★★☆☆ | Moderate — real algorithm, still finite/bounded                     | 1–2 days       |
| ★★★★☆ | Advanced — non-trivial linear algebra / numerics                    | Several days   |
| ★★★★★ | Hard / research-grade — iterative eigen-numerics or adaptive theory | 1+ week        |

> **Numeric-type note.** DSP filters and fixed-gain trackers should template all three
> types (`float`, `Q15`, `Q31`). Items marked *(float-first)* involve dynamic range or
> iterative conditioning that make fixed-point impractical initially — implement and
> validate in `float`, mirroring the dynamics-module convention.

---

## Master list (by priority)

| #  | Component                                            | Target module           | Difficulty |
|----|------------------------------------------------------|-------------------------|------------|
| 54 | Angle utilities (wrap, shortest difference, unwrap)  | `math`                  | ★☆☆☆☆      |
| 55 | Polynomial (cubic/quintic) point-to-point profile    | `motion_profiles` (new) | ★☆☆☆☆      |
| 56 | Trapezoidal (LSPB) profile + duration sync           | `motion_profiles` (new) | ★☆☆☆☆      |
| 57 | Friction compensation (Coulomb + viscous + Stribeck) | `controllers`           | ★☆☆☆☆      |
| 58 | Symplectic integrators (semi-implicit Euler, Verlet) | `solvers`               | ★☆☆☆☆      |
| 59 | Tridiagonal (Thomas) solver                          | `math`                  | ★☆☆☆☆      |
| 53 | SE(3) transform, twists, wrenches, adjoint, exp/log  | `math`                  | ★★☆☆☆      |
| 60 | S-curve (jerk-limited) profile                       | `motion_profiles` (new) | ★★☆☆☆      |
| 61 | Cubic spline interpolation (vector-valued)           | `math`                  | ★★☆☆☆      |
| 62 | Damped pseudo-inverse + null-space projector         | `solvers`               | ★★☆☆☆      |
| 63 | Incremental QR least squares (rank-revealing)        | `estimators/offline`    | ★★★☆☆      |
| 64 | Levenberg–Marquardt (retraction, pluggable damping)  | `optimization`          | ★★★☆☆      |
| 65 | Dense convex QP (Goldfarb–Idnani dual active set)    | `solvers`               | ★★★★☆      |

Items 53–65 are the generic building blocks of robotics-toolbox-cpp's roadmap (referenced there as
NT53–NT65); each also serves non-robotics consumers (motor control, navigation, calibration). All are
*(float-first)*.

Items 48–52 are the **evaluation & metrics primitives** — reusable quantities the per-family
unit-test reference [`TESTING.md`](TESTING.md) depends on but which
the library does not yet expose. Detailed below under
[Evaluation & metrics primitives](#evaluation--metrics-primitives).

## Tier 1 — Trivial primitives ★☆☆☆☆

### 54. Angle utilities
- **What:** `WrapToPi` `(−π, π]`, `WrapToTwoPi` `[0, 2π)`, shortest signed `AngleDifference`, and
  `UnwrapAngle` for continuous signals.
- **Embedded value:** One convention for every angle consumer — heading controllers, odometry, encoder
  unwrapping, analytical IK; replaces `ComplementaryFilter`'s private wrap.
- **Algorithm / paper:** range reduction with `fmod`; D. Goldberg, *ACM Computing Surveys*, 23(1), 1991.
- **Reuses:** `math::Fmod`.
- **Spec:** [roadmap/math/AngleUtilities](roadmap/math/AngleUtilities/implementation.md).

### 55. Polynomial point-to-point profile
- **What:** Cubic (position + velocity) or quintic (+ acceleration) time law between two states over a
  fixed duration; defines the shared `TrajectoryState` / `MotionLimits` types of `motion_profiles`.
- **Embedded value:** Coefficients solved once in closed form; each servo tick is a Horner evaluation.
- **Algorithm / paper:** M. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Ch. 5.
- **Reuses:** Scalar templates.
- **Spec:** [roadmap/motion_profiles/PolynomialProfile](roadmap/motion_profiles/PolynomialProfile/implementation.md).

### 56. Trapezoidal (LSPB) profile
- **What:** Minimum-time velocity/acceleration-limited rest-to-rest move, triangular for short moves, plus
  a fixed-duration plan for multi-axis synchronization.
- **Embedded value:** The standard servo-axis move (robot joints, CNC, conveyors, motor position loops).
- **Algorithm / paper:** L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and
  Robots* (2008), Ch. 3.
- **Reuses:** Item 55 types.
- **Spec:** [roadmap/motion_profiles/TrapezoidalProfile](roadmap/motion_profiles/TrapezoidalProfile/implementation.md).

### 57. Friction compensation
- **What:** Feedforward Coulomb + viscous + Stribeck friction per axis with a `tanh` zero-crossing and an
  output clamp.
- **Embedded value:** Removes stick-slip and tracking lag in geared drives and joints for one `exp` and one
  `tanh` per axis.
- **Algorithm / paper:** B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit, *Automatica*, 30(7), 1994.
- **Reuses:** `math::Vector`.
- **Spec:** [roadmap/controllers/FrictionCompensation](roadmap/controllers/FrictionCompensation/implementation.md).

### 58. Symplectic integrators
- **What:** Semi-implicit Euler and velocity Verlet for second-order systems `q̈ = a(q, q̇, u)`.
- **Embedded value:** Bounded energy error for simulated mechanical plants at one or two evaluations per
  step (RK4 needs four and still drifts).
- **Algorithm / paper:** E. Hairer, C. Lubich, G. Wanner, *Geometric Numerical Integration* (2006).
- **Reuses:** `math::Vector`; complements `RungeKuttaIntegrators`.
- **Spec:** [roadmap/solvers/SymplecticIntegrators](roadmap/solvers/SymplecticIntegrators/implementation.md).

### 59. Tridiagonal solver
- **What:** Thomas algorithm, factor once / solve many, pivot-failure reporting.
- **Embedded value:** `O(n)` instead of `O(n³)` for splines and implicit difference schemes.
- **Algorithm / paper:** L. H. Thomas (1949); G. Golub, C. Van Loan, *Matrix Computations*, §4.3.
- **Reuses:** Bounded `std::array` bands.
- **Spec:** [roadmap/math/TridiagonalSolver](roadmap/math/TridiagonalSolver/implementation.md).

---

## Tier 2 — Easy ★★☆☆☆

### 53. SE(3) transform, twists and wrenches
- **What:** Rigid transform (rotation + translation) with closed-form inverse, adjoint, twist/wrench
  transforms, exponential/logarithm (SO(3) and SE(3)) and a log-map pose error. 6-vectors are ordered
  linear part first: twists `(v; ω)`, wrenches `(f; n)`.
- **Embedded value:** The algebra every pose-level consumer shares — robot kinematics, Jacobians and
  task-space control in robotics-toolbox-cpp, drone attitude/position control, visual odometry. One
  type and one ordering convention prevent silent block-order mismatches between modules.
- **Algorithm / paper:** K. Lynch, F. Park, *Modern Robotics* (2017), Ch. 3; R. Murray, Z. Li,
  S. Sastry (1994), Ch. 2; J. Solà et al., "A micro Lie theory for state estimation in robotics" (2018).
- **Reuses:** `Geometry3D` (`RotationAboutAxis`, `SkewSymmetric`), `Quaternion`.
- **Spec:** [roadmap/math/SE3Transform](roadmap/math/SE3Transform/implementation.md).

### 60. S-curve (jerk-limited) profile
- **What:** Seven-segment jerk-bounded rest-to-rest move with all short-move cases in closed form.
- **Embedded value:** Continuous acceleration — far less vibration than a trapezoid for precision axes.
- **Algorithm / paper:** L. Biagiotti, C. Melchiorri (2008), §3.4.
- **Reuses:** Items 55 (types) and 56 (synchronization scheme).
- **Spec:** [roadmap/motion_profiles/SCurveProfile](roadmap/motion_profiles/SCurveProfile/implementation.md).

### 61. Cubic spline interpolation
- **What:** C² vector-valued spline through `K` knots, clamped or natural ends, shared factorization across
  channels.
- **Embedded value:** Smooth interpolation of waypoint lists and calibration tables in linear time on fixed
  storage.
- **Algorithm / paper:** C. de Boor, *A Practical Guide to Splines* (2001), Ch. IV.
- **Reuses:** Item 59.
- **Spec:** [roadmap/math/CubicSpline](roadmap/math/CubicSpline/implementation.md).

### 62. Damped pseudo-inverse and null-space projector
- **What:** `A⁺_λ·b` and the exact projector `I − A⁺A` for any `Rows × Cols` matrix from one SVD.
- **Embedded value:** Singularity-robust inversion for redundant robots, over-actuated control allocation
  and regularized calibration; the projector exposes spare actuation to a secondary objective.
- **Algorithm / paper:** C. Wampler, *IEEE Trans. SMC*, 16(1), 1986; A. Liégeois, *IEEE Trans. SMC*,
  7(12), 1977.
- **Reuses:** `SingularValueDecomposition` (tall and wide).
- **Spec:** [roadmap/solvers/DampedPseudoInverse](roadmap/solvers/DampedPseudoInverse/implementation.md).

---

## Tier 3 — Moderate ★★★☆☆

### 63. Incremental QR least squares
- **What:** Streaming Givens-QR accumulation of an unbounded number of rows into a `P×P` factor, with a
  rank-revealing SVD solve (minimum-norm solution, identifiable directions, residual RMS).
- **Embedded value:** Parameter identification from long logs in fixed memory without squaring the
  condition number.
- **Algorithm / paper:** G. Golub, C. Van Loan, §5.1.8 and §5.5; Å. Björck (1996), Ch. 2.
- **Reuses:** `GivensRotation`, `SingularValueDecomposition`.
- **Spec:** [roadmap/estimators/offline/IncrementalQrLeastSquares](roadmap/estimators/offline/IncrementalQrLeastSquares/implementation.md).

### 64. Levenberg–Marquardt
- **What:** Damped Gauss–Newton for small nonlinear least squares with a problem-supplied retraction
  (vectors, bounds, rotations) and a pluggable damping policy.
- **Embedded value:** One bounded, heap-free solver for IK, parallel-robot forward kinematics, calibration
  and scan matching.
- **Algorithm / paper:** K. Levenberg (1944); D. Marquardt (1963); J. Nocedal, S. Wright (2006), Ch. 10.
- **Reuses:** `LuDecomposition`.
- **Spec:** [roadmap/optimization/LevenbergMarquardt](roadmap/optimization/LevenbergMarquardt/implementation.md).

---

## Tier 4 — Advanced ★★★★☆

### 65. Dense convex QP
- **What:** Goldfarb–Idnani dual active-set solver for strictly convex QPs with equality and inequality
  constraints; finite, bounded iterations.
- **Embedded value:** Exact constrained allocation (cable tensions, thrusters), task-priority control and
  constrained MPC without heuristics.
- **Algorithm / paper:** D. Goldfarb, A. Idnani, *Mathematical Programming*, 27(1), 1983; M. J. D. Powell
  (1985).
- **Reuses:** `CholeskyDecomposition`, `TriangularSolve`, `GivensRotation`.
- **Spec:** [roadmap/solvers/ActiveSetQp](roadmap/solvers/ActiveSetQp/implementation.md).

---

## Evaluation & metrics primitives

Reusable quantities that quantify *how well* an algorithm behaves — the properties the per-family
unit-test reference [`TESTING.md`](TESTING.md) asks tests to assert.
Today [`math/Statistics.hpp`](numerical/math/Statistics.hpp) already provides `Mean`, `Variance`,
`MeanSquaredError`, `RootMeanSquaredError`, `MeanAbsoluteError`, `RSquaredScore`, `AutoCorrelation`,
and [`control_analysis/FrequencyResponse`](numerical/control_analysis/FrequencyResponse.hpp) provides
magnitude/phase. The items below are the missing pieces. All are **float-only**, no-heap, and operate
on bounded `math::Vector`/`math::Matrix` inputs; tests are `TEST_F` on `float`.

> **Test-only helpers (not production components).** A ULP/relative-error comparator and a
> finite-difference **gradient check** (for `optimization/`) are pure test
> utilities — add them to the `numerical.math_test_helper` INTERFACE library
> ([`numerical/math/test_doubles/`](numerical/math/test_doubles/)), not to `numerical/` production code.

## Per-component implementation checklist

Every new component should follow the established repository conventions:

- [ ] Header-only template supporting `float` / `math::Q15` / `math::Q31` (or *float-first* where noted)
- [ ] No `#pragma GCC optimize` / `optimize` attribute; `OPTIMIZE_FOR_SPEED` on hot paths
- [ ] No heap, no recursion, bounded containers (`infra::BoundedVector`, `std::array`)
- [ ] `static_assert` on supported types and dimensions
- [ ] Typed tests (`TYPED_TEST`) for multi-type components; `TEST_F` for single-type; `StrictMock` only
- [ ] Design-first `doc/<domain>/<Name>.md` following `doc/TEMPLATE.md`; add its row to
      `doc/<domain>/README.md` and `README.md`'s Documentation table (the booklet regenerates from
      these tables in CI — no manual booklet edit)
- [ ] Explicit template instantiation `.cpp` guarded by `NUMERICAL_TOOLBOX_COVERAGE_BUILD` + `numerical_add_coverage_sources`
- [ ] `CMakeLists.txt` via `numerical_add_header_library()` / `${NUMERICAL_VISIBILITY}`
- [ ] Simulator + `.vscode/launch.json` entry where a visual demo adds value
