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

| #  | Component                                            | Target module             | Difficulty |
|----|------------------------------------------------------|---------------------------|------------|
| 53 | SE(3) transform, twists, wrenches, adjoint, exp/log  | `math`                    | ★★☆☆☆      |

Items 48–52 are the **evaluation & metrics primitives** — reusable quantities the per-family
unit-test reference [`TESTING.md`](TESTING.md) depends on but which
the library does not yet expose. Detailed below under
[Evaluation & metrics primitives](#evaluation--metrics-primitives).

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
