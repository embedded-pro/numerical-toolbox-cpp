# Quaternion — Implementation Pseudocode

> Roadmap ref: #18 (Tier 3) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class Quaternion:
    T w, x, y, z            # w + xi + yj + zk; unit norm for rotations
```

Store as four scalars (or reuse `Vector<T, 4>`). For a rotation every component lies in
[-1, 1].

## Interface

```
Quaternion(T w = 1, T x = 0, T y = 0, T z = 0)
static Quaternion Identity()
static Quaternion FromAxisAngle(Vector3<T> axis, T angle)
static Quaternion FromRotationMatrix(Matrix3<T> R)
Matrix3<T>  ToRotationMatrix()
Vector3<T>  ToEuler()                        # roll-pitch-yaw (ZYX)
Quaternion  operator*(Quaternion rhs)        # Hamilton product; hot path
Quaternion  Conjugate()
Quaternion  Inverse()
T           Norm() / T SquaredNorm()
Quaternion& Normalize()
Vector3<T>  Rotate(Vector3<T> v)             # hot path
static Quaternion Slerp(Quaternion a, Quaternion b, T t)
```

## Algorithm (pseudocode)

```
function multiply(a, b):                     # OPTIMIZE_FOR_SPEED — Hamilton product
    w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
    y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
    z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    return Quaternion(w, x, y, z)

function Rotate(v):                           # OPTIMIZE_FOR_SPEED
    # v' = q * (0,v) * q⁻¹  via the cheaper cross-product form
    u = (x, y, z)
    t = 2 * CrossProduct(u, v)
    return v + w*t + CrossProduct(u, t)

function Slerp(a, b, t):
    d = dot(a, b)                            # cosine of the half-angle
    if d < 0: b = -b; d = -d                 # take the short arc (double cover)
    if d > 0.9995: return Normalize(a + t*(b - a))   # near-parallel -> lerp
    theta = acos(d)
    return (sin((1-t)*theta)*a + sin(t*theta)*b) / sin(theta)

function Normalize():
    n = sqrt(w² + x² + y² + z²)
    divide every component by n
```

## Complexity & memory

- Product / rotate: `O(1)` — 16 / 15 multiply-adds, no loops.
- SLERP: `O(1)` plus one `acos` and two `sin`.
- Memory: four scalars per quaternion; entirely on the stack, no heap.

## Numerical / embedded notes

- **Renormalize periodically** — repeated products drift off the unit sphere; renormalize when
  `|SquaredNorm − 1| > eps` rather than every step to save cycles.
- Double cover: `q` and `−q` are the same rotation — the SLERP sign flip selects the short path.
- Prefer the cross-product `Rotate` form (15 MACs) when rotating a single vector; build
  `ToRotationMatrix` only when many vectors share one rotation.
- `FromRotationMatrix` must branch on the largest diagonal term to avoid dividing by a near-zero.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/Quaternion.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `operator*`/`Rotate`, and
  `extern template class Quaternion<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/Quaternion.cpp` → `template class Quaternion<float>;`
- Test: `numerical/math/test/TestQuaternion.cpp`
- Doc: `doc/math/Quaternion.md` (new folder; math currently has no doc pages)
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestQuaternion.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".
