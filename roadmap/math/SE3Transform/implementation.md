# SE(3) Transform — Implementation Pseudocode

> Roadmap ref: #M6 (Tier 2) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                           # static_assert(std::is_floating_point_v<T>); instantiated for float
class SE3:
    Matrix3<T>  R          # rotation, on SO(3)
    Vector3<T>  p          # translation
    # equivalently the 4×4 homogeneous matrix [[R, p],[0 0 0 1]]

# spatial 6-vectors reuse Vector<T, 6>:
#   Twist  = (ω; v)   angular over linear velocity
#   Wrench = (m; f)   moment over force
```

## Interface

```
SE3()                                          # identity
SE3(Matrix3<T> R, Vector3<T> p)
SquareMatrix<T, 4> Homogeneous()               # 4×4 form
SE3          operator*(SE3 rhs)                # compose transforms; hot path
SE3          Inverse()
Vector3<T>   Apply(Vector3<T> point)           # transform a point; hot path
Matrix<T, 6, 6> Adjoint()                      # maps twists/wrenches between frames
static SE3   Exp(Vector<T, 6> xi, T theta)     # exp: se(3) -> SE(3)
Vector<T, 6> Log()                             # log: SE(3) -> se(3)
```

## Algorithm (pseudocode)

```
function multiply(a, b):                        # OPTIMIZE_FOR_SPEED
    return SE3(a.R * b.R, a.R * b.p + a.p)

function Inverse():                             # closed-form rigid inverse (no matrix solve)
    return SE3(Transpose(R), -Transpose(R) * p)

function Apply(point):
    return R * point + p

function Adjoint():                             # 6×6:  [[R, 0],[ [p]× R, R ]]
    upper-left  = R
    lower-left  = SkewSymmetric(p) * R          # reuse Geometry3D::SkewSymmetric
    lower-right = R
    upper-right = 0

function Exp(xi = (ω; v), theta):               # matrix exponential on se(3)
    if VectorNorm(ω) == 0: return SE3(I, v*theta)         # pure translation
    R = RotationAboutAxis(ω, theta)             # Rodrigues (reuse Geometry3D)
    G = I*theta + (1-cosθ)*SkewSymmetric(ω) + (θ-sinθ)*SkewSymmetric(ω)²   # left Jacobian
    return SE3(R, G * v)

function Log():                                 # inverse of Exp
    (ω, theta) = axisAngleFromRotation(R)       # log on SO(3)
    v = InverseLeftJacobian(theta) * p
    return concat(ω*theta, v*theta)
```

## Complexity & memory

- Compose / inverse / apply: `O(1)` — fixed 3×3 and 3-vector arithmetic, no loops.
- Adjoint / Exp / Log: `O(1)` — a handful of 3×3 products; `Log` adds one `acos`.
- Memory: a 3×3 plus a 3-vector per transform (or one 4×4); all stack-allocated, no heap.

## Numerical / embedded notes

- Use the **closed-form rigid inverse** `(Rᵀ, −Rᵀp)`; never run a general 4×4 matrix solve.
- Keep `R` on `SO(3)`: reorthonormalize after long product chains (drift accumulates like quaternions).
- `Exp`/`Log` need `sinθ/θ` and `(1−cosθ)/θ²`; use series expansions near `θ→0` to avoid `0/0`.
- Twist ordering here is `(ω; v)` (angular first); the adjoint block layout must match that choice.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/SE3Transform.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `operator*`/`Apply`, and
  `extern template class SE3<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/SE3Transform.cpp` → `template class SE3<float>;`
- Test: `numerical/math/test/TestSE3Transform.cpp`
- Doc: `doc/math/SE3Transform.md` (new folder; math currently has no doc pages)
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestSE3Transform.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
