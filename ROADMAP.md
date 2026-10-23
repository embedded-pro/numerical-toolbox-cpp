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

| #  | Component                                             | Target module           | Difficulty |
|----|-------------------------------------------------------|-------------------------|------------|
| 56 | Trapezoidal (LSPB) profile + duration sync            | `motion_profiles` (new) | ★☆☆☆☆      |
| 53 | SE(3) transform, twists, wrenches, adjoint, exp/log   | `math`                  | ★★☆☆☆      |
| 60 | S-curve (jerk-limited) profile                        | `motion_profiles` (new) | ★★☆☆☆      |
| 61 | Cubic spline interpolation (vector-valued)            | `math`                  | ★★☆☆☆      |
| 62 | Damped pseudo-inverse + null-space projector          | `solvers`               | ★★☆☆☆      |
| 66 | SE(2) transform, twists, exp/log, pose error          | `math`                  | ★★☆☆☆      |
| 67 | Kabsch / Umeyama point-set alignment                  | `solvers`               | ★★☆☆☆      |
| 63 | Incremental QR least squares (rank-revealing)         | `estimators/offline`    | ★★★☆☆      |
| 64 | Levenberg–Marquardt (retraction, pluggable damping)   | `optimization`          | ★★★☆☆      |
| 68 | Lie-group Jacobians and ⊕/⊖ for SO(3), SE(3), SE(2)   | `math`                  | ★★★☆☆      |
| 69 | Control allocation (redistributed pseudo-inverse, QP) | `controllers`           | ★★★☆☆      |
| 70 | Particle filter (bounded, systematic resampling)      | `filters/active`        | ★★★☆☆      |
| 65 | Dense convex QP (Goldfarb–Idnani dual active set)     | `solvers`               | ★★★★☆      |
| 71 | Error-state (multiplicative) EKF on Lie groups        | `filters/active`        | ★★★★☆      |
| 72 | Minimum-derivative (min-snap) piecewise polynomials   | `motion_profiles` (new) | ★★★★☆      |
| 73 | Online jerk-limited trajectory from arbitrary state   | `motion_profiles` (new) | ★★★★★      |

Items 53–73 are the generic building blocks of robotics-toolbox-cpp's roadmap (referenced there as
NT53–NT73); each also serves non-robotics consumers (motor control, navigation, calibration). All are
*(float-first)*. Items 66–73 serve its mobile, aerial, legged, marine and localization families and
have no spec yet.

Items 48–52 are the **evaluation & metrics primitives** — reusable quantities the per-family
unit-test reference [`TESTING.md`](TESTING.md) depends on but which
the library does not yet expose. Detailed below under
[Evaluation & metrics primitives](#evaluation--metrics-primitives).

## Tier 1 — Trivial primitives ★☆☆☆☆

### 56. Trapezoidal (LSPB) profile
- **What:** Minimum-time velocity/acceleration-limited rest-to-rest move, triangular for short moves, plus
  a fixed-duration plan for multi-axis synchronization.
- **Embedded value:** The standard servo-axis move (robot joints, CNC, conveyors, motor position loops).
- **Algorithm / paper:** L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and
  Robots* (2008), Ch. 3.
- **Reuses:** Item 55 types.
- **Spec:** [roadmap/motion_profiles/TrapezoidalProfile](roadmap/motion_profiles/TrapezoidalProfile/implementation.md).

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

### 66. SE(2) transform
- **What:** Planar rigid transform `(R(θ), p)` with composition, closed-form inverse, adjoint, exp/log of
  planar twists `(vx, vy, ω)` and a pose error, ordered linear part first like item 53.
- **Embedded value:** The pose type of every ground vehicle and planar mechanism — odometry, path
  tracking, scan matching — at a fraction of SE(3)'s cost.
- **Algorithm / paper:** J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in
  robotics" (2018), Appendix C.
- **Reuses:** Item 54 (angle wrapping).

### 67. Kabsch / Umeyama alignment
- **What:** Least-squares rotation, translation and optional scale between two corresponding point sets
  (orthogonal Procrustes), with the reflection fix, in 2-D and 3-D.
- **Embedded value:** The inner step of ICP scan matching, hand-eye and frame calibration, motion-capture
  and fiducial registration.
- **Algorithm / paper:** W. Kabsch, *Acta Cryst.* A32, 1976; S. Umeyama, *IEEE T-PAMI*, 13(4), 1991.
- **Reuses:** `SingularValueDecomposition`.

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

### 68. Lie-group Jacobians and ⊕/⊖
- **What:** Left and right Jacobians (and inverses) of SO(3), SE(3) and SE(2), the `⊕`/`⊖` operators
  on the tangent space, and adjoint-based covariance transport between frames.
- **Embedded value:** What every on-manifold estimator and optimizer needs to linearize correctly —
  error-state Kalman filters, IMU preintegration, pose-graph and calibration solvers.
- **Algorithm / paper:** J. Solà et al. (2018); T. Barfoot, *State Estimation for Robotics* (2017), Ch. 7.
- **Reuses:** Items 53 and 66.

### 69. Control allocation
- **What:** Map a desired generalized force (thrust/torque, wrench) to `m` actuator commands with limits:
  weighted pseudo-inverse with iterative redistribution of saturated actuators, and an exact QP variant.
- **Embedded value:** Multirotor and hexacopter mixers, marine thruster allocation, over-actuated
  vehicles and redundant drives — graceful saturation instead of clipping each motor.
- **Algorithm / paper:** T. A. Johansen, T. I. Fossen, "Control allocation — A survey," *Automatica*,
  49(5), 2013; J. Bordignon, *Constrained control allocation* (PhD thesis, Virginia Tech, 1996).
- **Reuses:** Items 62 and 65.

### 70. Particle filter
- **What:** Sequential importance resampling with a compile-time particle count, systematic resampling
  triggered by the effective sample size, and weighted-mean / MAP estimates.
- **Embedded value:** Multimodal, non-Gaussian estimation — Monte Carlo localization, terrain-aided
  navigation, fault detection — on fixed memory.
- **Algorithm / paper:** N. Gordon, D. Salmond, A. Smith, *IEE Proc. F*, 140(2), 1993; A. Doucet,
  A. Johansen, "A tutorial on particle filtering and smoothing" (2009).
- **Reuses:** A caller-injected deterministic PRNG; `math::Statistics`.

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

### 71. Error-state Kalman filter on Lie groups
- **What:** Multiplicative EKF whose nominal state lives on a group (quaternion/SO(3), SE(3)) and whose
  error state lives in the tangent space; predict, update, reset with the right Jacobians.
- **Embedded value:** The standard INS/AHRS and legged/aerial state estimator — no quaternion
  normalization hacks, covariance consistent with the manifold.
- **Algorithm / paper:** J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508
  (2017); F. L. Markley, *J. Guidance, Control, and Dynamics*, 26(2), 2003.
- **Reuses:** Items 53 and 68, `KalmanFilterBase`.

### 72. Minimum-derivative piecewise polynomials
- **What:** Piecewise polynomials through timed waypoints minimizing the integral of a chosen derivative
  squared (jerk, snap), per axis, with optional corridor constraints; closed-form KKT solve without
  inequalities, QP with them.
- **Embedded value:** Smooth references for multirotors (minimum snap), manipulators and CNC paths with
  continuity up to the third or fourth derivative.
- **Algorithm / paper:** D. Mellinger, V. Kumar, *ICRA* 2011; C. Richter, A. Bry, N. Roy, *ISRR* 2013.
- **Reuses:** Item 65, `CholeskyDecomposition`.

---

## Tier 5 — Hard / research-grade ★★★★★

### 73. Online jerk-limited trajectory from arbitrary state
- **What:** Time-optimal single-axis trajectory from any current state (position, velocity, acceleration)
  to any target state under velocity, acceleration and jerk limits, plus multi-axis time
  synchronization — recomputable every control cycle.
- **Embedded value:** Reactive motion (target changes mid-move, emergency re-plans) for robot joints and
  servo axes without the rest-to-rest restriction of item 60.
- **Algorithm / paper:** T. Kröger, F. Wahl, *IEEE T-RO*, 26(1), 2010; L. Berscheid, T. Kröger,
  "Jerk-limited real-time trajectory generation with arbitrary target states," *RSS* 2021.
- **Reuses:** Items 55, 56 and 60 (types, synchronization scheme), `DurandKerner` for the polynomial
  roots of the timing profiles.

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
