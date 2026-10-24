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
| 66 | SE(2) transform, twists, exp/log, pose error          | `math`                  | ★★☆☆☆      |
| 67 | Kabsch / Umeyama point-set alignment                  | `solvers`               | ★★☆☆☆      |
| 68 | Lie-group Jacobians and ⊕/⊖ for SO(3), SE(3), SE(2)   | `math`                  | ★★★☆☆      |
| 69 | Control allocation (redistributed pseudo-inverse, QP) | `controllers`           | ★★★☆☆      |
| 70 | Particle filter (bounded, systematic resampling)      | `filters/active`        | ★★★☆☆      |
| 71 | Error-state (multiplicative) EKF on Lie groups        | `filters/active`        | ★★★★☆      |
| 72 | Minimum-derivative (min-snap) piecewise polynomials   | `motion_profiles` (new) | ★★★★☆      |
| 73 | Online jerk-limited trajectory from arbitrary state   | `motion_profiles` (new) | ★★★★★      |

Items 66–73 are generic building blocks of robotics-toolbox-cpp's roadmap (referenced there as
NT66–NT73; NT53–NT65 are deployed). They serve its mobile, aerial, legged, marine and localization
families and also have non-robotics consumers. Item 66 has a spec under [`roadmap/`](roadmap/README.md);
items 67–73 have none yet. All are *(float-first)*.

Items 48–52 are the **evaluation & metrics primitives** — reusable quantities the per-family
unit-test reference [`TESTING.md`](TESTING.md) depends on but which
the library does not yet expose. Detailed below under
[Evaluation & metrics primitives](#evaluation--metrics-primitives).

## Tier 2 — Easy ★★☆☆☆

### 66. SE(2) transform
- **What:** Planar rigid transform `(R(θ), p)` with composition, closed-form inverse, adjoint, exp/log of
  planar twists `(vx, vy, ω)` and a pose error, ordered linear part first like item 53.
- **Embedded value:** The pose type of every ground vehicle and planar mechanism — odometry, path
  tracking, scan matching — at a fraction of SE(3)'s cost.
- **Algorithm / paper:** J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in
  robotics" (2018), Appendix C.
- **Reuses:** Item 53 (ordering convention, `ToSE3`).
- **Spec:** [roadmap/math/SE2Transform](roadmap/math/SE2Transform/implementation.md).

### 67. Kabsch / Umeyama alignment
- **What:** Least-squares rotation, translation and optional scale between two corresponding point sets
  (orthogonal Procrustes), with the reflection fix, in 2-D and 3-D.
- **Embedded value:** The inner step of ICP scan matching, hand-eye and frame calibration, motion-capture
  and fiducial registration.
- **Algorithm / paper:** W. Kabsch, *Acta Cryst.* A32, 1976; S. Umeyama, *IEEE T-PAMI*, 13(4), 1991.
- **Reuses:** `SingularValueDecomposition`.

---

## Tier 3 — Moderate ★★★☆☆

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
