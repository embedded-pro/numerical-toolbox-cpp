# SE(3) Transform, Twists and Wrenches — Implementation Pseudocode

> Roadmap ref: #53 (Tier 2) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

Rigid-body transform plus the twist/wrench algebra that every pose-level consumer needs (robot
kinematics and control in robotics-toolbox-cpp, SLAM, drone navigation). It fixes the **one ordering
convention** shared by all consumers: 6-vectors are ordered **linear part first** — twists `(v; ω)`,
wrenches `(f; n)` — matching the geometric Jacobian with linear rows first and the tangent ordering of
Sophus and manif. Lynch & Park and Murray et al. use `(ω; v)`; the two only permute the blocks.

## Data structures

```cpp
template<typename T> using Vector6 = math::Vector<T, 6>      # twist (v; ω) or wrench (f; n)
template<typename T> using Matrix6 = math::SquareMatrix<T, 6>

template<typename T>                  # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SE3Transform:
    Matrix3<T> R                      # rotation, orthonormal, det = +1
    Vector3<T> p                      # translation
    # equivalently the 4×4 homogeneous matrix [[R, p],[0 0 0 1]]
```

## Interface

```text
static SE3Transform Identity()
SE3Transform operator*(const SE3Transform& rhs) const        # composition; hot path
SE3Transform Inverse() const                                 # closed form, no matrix solve
Vector3<T>   Apply(const Vector3<T>& point) const            # R·x + p; hot path
Vector3<T>   Rotate(const Vector3<T>& direction) const       # R·x
SquareMatrix<T, 4> Homogeneous() const
Matrix6<T>   Adjoint() const                                 # twist map for (v; ω)
Vector6<T>   TransformTwist(const Vector6<T>& twist) const   # Adjoint()·twist
Vector6<T>   TransformWrench(const Vector6<T>& wrench) const # dual map, see below

static SE3Transform Exp(const Vector6<T>& twist, T theta)    # e^{[ξ]θ}; hot path
Vector6<T>          Log() const                              # inverse of Exp with theta = 1
static Vector3<T>   LogRotation(const Matrix3<T>& R)         # SO(3) log: rotation vector, |·| ≤ π
static Vector6<T>   PoseError(const SE3Transform& target, const SE3Transform& current)
```

## Algorithm (pseudocode)

```text
function operator*(rhs):   return { R·rhs.R, R·rhs.p + p }
function Inverse():        return { Rᵀ, −Rᵀ·p }

function Adjoint():                         # V_A = Ad_T · V_B for V = (v; ω)
    return [[ R,  Skew(p)·R ],              # reuse Geometry3D::SkewSymmetric
            [ 0,  R         ]]

function TransformWrench(w = (f; n)):       # power-consistent dual of Adjoint
    fA = R·f
    nA = R·n + CrossProduct(p, R·f)
    return (fA; nA)

function Exp(ξ = (v; ω), θ):                # OPTIMIZE_FOR_SPEED
    if ‖ω‖ < ε:                             # pure translation
        return { I, v·θ }
    # general twist: fold the magnitude of ω into θ so the axis is unit
    s = ‖ω‖;  ω̂ = ω / s;  v̂ = v / s;  φ = θ·s
    R = RotationAboutAxis(ω̂, φ)             # reuse Geometry3D (Rodrigues)
    G = I·φ + (1 − cos φ)·Skew(ω̂) + (φ − sin φ)·Skew(ω̂)²   # left Jacobian · φ
    return { R, G·v̂ }

function LogRotation(R):                    # rotation vector ω̂·φ, φ ∈ [0, π]
    φ = atan2(‖Vee(R − Rᵀ)‖/2, (trace(R) − 1)/2)            # accurate near 0 and π, unlike acos
    if φ < ε:                               # small angle: first-order
        return Vee(R − Rᵀ) / 2
    if π − φ < ε:                           # sin φ → 0: use the diagonal form
        ω̂ = axis from the largest of sqrt((R_ii + 1) / 2), signs from the off-diagonals
    else:
        ω̂ = Vee(R − Rᵀ) / (2 sin φ)
    return ω̂·φ

function Log():                             # returns (v; ω) with ‖ω‖ = rotation angle
    ω = LogRotation(R);  φ = ‖ω‖
    if φ < ε: return (p; ω)
    ω̂ = ω / φ
    G⁻¹ = I/φ − Skew(ω̂)/2 + (1/φ − cot(φ/2)/2)·Skew(ω̂)²
    return (G⁻¹·p·φ; ω)

function PoseError(target, current):        # base-frame error, matches a linear-rows-first Jacobian
    eP = target.p − current.p
    eR = LogRotation(target.R · current.Rᵀ)                  # |eR| ≤ π, short way round
    return (eP; eR)
```

## Complexity & memory

- Compose / inverse / apply / rotate: `O(1)` — a handful of 3×3 products.
- `Exp` / `Log`: `O(1)` — one `sin`/`cos` pair (plus `atan2` for `Log`).
- Memory: 12 scalars per transform; `Matrix6` only when an adjoint is requested. No heap.

## Numerical / embedded notes

- **Convention (normative):** twists `(v; ω)`, wrenches `(f; n)`; `PoseError` returns
  `(position; rotation-vector)`. robotics-toolbox-cpp's geometric Jacobian, controllers and pose IK
  depend on this ordering; changing it is a breaking change for that consumer.
- Use the **closed-form rigid inverse** `(Rᵀ, −Rᵀp)`; never run a general 4×4 solve.
- `Log` must be the true logarithm: `2·vec(quaternion)` equals `2 sin(φ/2)·ω̂`, which is only a
  small-angle approximation of `φ·ω̂`. Rotations beyond `π` are re-expressed the short way, so `|eR| ≤ π`.
- Handle both singular regions of `Log` (`φ → 0` and `φ → π`) explicitly; derive `φ` with `atan2`, not
  `acos`, which loses precision near ±1 in `float`.
- Composition drifts off SO(3) over long chains of products in `float`; re-orthonormalize (Gram–Schmidt
  or via `Quaternion` normalization) when a transform is integrated over time, not after single
  compositions.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/math/SE3Transform.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `operator*`/`Apply`/`Exp`, and `extern template struct SE3Transform<float>;` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/SE3Transform.cpp` → `template struct SE3Transform<float>;`
- Test: `numerical/math/test/TestSE3Transform.cpp`
- Doc: `doc/math/SE3Transform.md` (per `doc/TEMPLATE.md`) + row in `doc/math/README.md`
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestSE3Transform.cpp` → `numerical.math_test`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
