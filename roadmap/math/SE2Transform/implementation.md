# SE(2) Transform, Planar Twists and Wrenches — Implementation Pseudocode

> Roadmap ref: #66 (Tier 2) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

Planar rigid transform with the twist/wrench algebra of item 53, reduced to the plane: the pose type of
ground vehicles and planar mechanisms (odometry, path tracking, scan matching). It follows item 53's
convention: 3-vectors are ordered **linear part first**, twists `(vx, vy, ω)` and wrenches `(fx, fy, τ)`.
The rotation is stored as a unit complex number `(cos θ, sin θ)`, as Sophus and manif do. Composition,
`Apply` and `Inverse` then need no trigonometry; only the conversion to and from a heading does.

## Data structures

```cpp
template<typename T> using Vector2 = math::Vector<T, 2>
template<typename T> using Matrix2 = math::SquareMatrix<T, 2>
# planar twist (vx, vy, ω), planar wrench (fx, fy, τ) and the pose triple (x, y, θ) are math::Vector3<T>

template<typename T>                  # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SE2Transform:
    T cosine{ 1 }                     # unit complex rotation (cos θ, sin θ)
    T sine{ 0 }
    Vector2<T> translation{}
    # equivalently the 3×3 homogeneous matrix [[R(θ), p],[0 0 1]]
```

## Interface

```text
static SE2Transform Identity()
static SE2Transform FromPose(T x, T y, T theta)              # the only trig on construction
Vector3<T>   Pose() const                                    # (x, y, θ), θ = atan2(sin, cos) ∈ (−π, π]
T            Heading() const
Matrix2<T>   Rotation() const
friend SE2Transform operator*(const SE2Transform& lhs, const SE2Transform& rhs)   # hot path
SE2Transform Inverse() const                                 # closed form
Vector2<T>   Apply(const Vector2<T>& point) const            # R·x + p; hot path
Vector2<T>   Rotate(const Vector2<T>& direction) const       # R·x
SquareMatrix<T, 3> Homogeneous() const
Matrix3<T>   Adjoint() const                                 # twist map for (vx, vy, ω)
Vector3<T>   TransformTwist(const Vector3<T>& twist) const   # Adjoint()·twist
Vector3<T>   TransformWrench(const Vector3<T>& wrench) const # dual map, see below
SE2Transform Normalized() const                              # rescale (cos, sin) to unit length
SE3Transform<T> ToSE3() const                                # rotation about z, translation (x, y, 0)

static SE2Transform Exp(const Vector3<T>& twist, T theta)    # e^{[ξ]θ}; θ is the elapsed time for a velocity twist; hot path
Vector3<T>          Log() const                              # inverse of Exp with theta = 1
static Vector3<T>   PoseError(const SE2Transform& target, const SE2Transform& current)
```

`operator*` is a hidden friend, as in item 53. The heading is never stored, so `Pose` always returns it
wrapped. Only `FromPose` checks for a unit rotation: an aggregate-initialised transform must be given a
unit `(cosine, sine)`.

## Algorithm (pseudocode)

```text
function FromPose(x, y, θ):  return { cos θ, sin θ, (x, y) }
function Pose():             return (p.x, p.y, atan2(sine, cosine))
function Rotation():         return [[c, −s], [s, c]]

function operator*(a, b):                   # complex product + rotated offset
    return { a.c·b.c − a.s·b.s,  a.s·b.c + a.c·b.s,  a.R·b.p + a.p }
function Inverse():          return { c, −s, −Rᵀ·p }

function Adjoint():                         # V_A = Ad_T · V_B for V = (vx, vy, ω)
    return [[ c, −s,  p.y ],
            [ s,  c, −p.x ],
            [ 0,  0,  1   ]]

function TransformWrench(w = (fx, fy, τ)):  # power-consistent dual of Adjoint
    f_A = R·(fx, fy)
    return (f_A; τ + p.x·f_A.y − p.y·f_A.x)

function Normalized():
    k = 1 / Hypot(c, s);  return { c·k, s·k, p }

function Exp(ξ = (vx, vy, ω), θ):           # OPTIMIZE_FOR_SPEED
    φ = ω·θ
    if |φ| < 0.1:                           # series, no 0/0 and no cancellation
        A = 1 − φ²/6 + φ⁴/120               # sin φ / φ
        B = φ/2 − φ³/24 + φ⁵/720            # (1 − cos φ) / φ
    else:
        A = sin φ / φ
        B = 2·sin²(φ/2) / φ                 # half-angle form of (1 − cos φ)/φ
    (ux, uy) = (vx·θ, vy·θ)
    return { cos φ, sin φ, (A·ux − B·uy, B·ux + A·uy) }    # p = V(φ)·u, V = [[A, −B], [B, A]]

function Log():                             # returns (vx, vy, ω) with ω = Heading() ∈ (−π, π]
    φ = atan2(s, c);  h = φ/2
    if |φ| < 0.1:  α = 1 − φ²/12 − φ⁴/720  # h·cot h
    else:          α = h·cos h / sin h      # finite on the whole branch: α(±π) = 0
    return (α·p.x + h·p.y,  −h·p.x + α·p.y,  φ)            # V(φ)⁻¹·p

function PoseError(target, current):        # base-frame error, like item 53's PoseError
    eθ = atan2(target.s·current.c − target.c·current.s,
               target.c·current.c + target.s·current.s)     # heading of target·currentᵀ, short way
    return (target.p − current.p; eθ)
```

## Complexity & memory

- Compose / inverse / apply / rotate / adjoint: `O(1)`, at most 8 multiplies, no trigonometry.
- `Exp`: one `sin`/`cos` pair plus one `sin` (half angle); `Log`: one `atan2` and one `sin`/`cos` pair.
- Memory: 4 scalars per transform. No heap.

## Numerical / embedded notes

- **Convention (normative):** twists `(vx, vy, ω)`, wrenches `(fx, fy, τ)`, `PoseError` and `Log` return
  linear part first. robotics-toolbox-cpp's wheeled models, odometry and trackers depend on this ordering.
- `Log` has no singular point: on the branch `φ ∈ (−π, π]` the coefficient `(φ/2)·cot(φ/2)` stays finite
  and reaches 0 at `±π`. It does return the **wrapped** angle, so `Log(Exp(ξ, θ)) = ξ·θ` only while
  `|ω·θ| < π`. Odometry that must count full turns integrates `Exp` and does not round-trip through `Log`.
- The series threshold `|φ| < 0.1` mirrors item 53. The truncation error there is `≈1.6e-10`, below
  `float` resolution, so `Exp` is continuous across the switch.
- Composing in `float` lets `cos² + sin²` drift from 1 by roughly one ulp per product. A transform that is
  integrated over time (odometry) calls `Normalized` every step, which costs one `Hypot`. Single compositions
  need no renormalisation.
- `FromPose` accepts any angle; no wrapping is needed because only `cos θ` and `sin θ` are stored.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/math/SE2Transform.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `operator*`/`Apply`/`Exp`, and `extern template struct SE2Transform<float>;` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`. Includes `SE3Transform.hpp` for `ToSE3` and `Vector3`.
- Coverage: `numerical/math/SE2Transform.cpp` → `template struct SE2Transform<float>;`
- Test: `numerical/math/test/TestSE2Transform.cpp`
- Doc: `doc/math/SE2Transform.md` (per `doc/TEMPLATE.md`) + row in `doc/math/README.md`
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestSE2Transform.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
