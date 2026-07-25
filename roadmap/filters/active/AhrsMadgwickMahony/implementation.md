# Madgwick / Mahony AHRS — Implementation Pseudocode

> Roadmap ref: #33 (Tier 4) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, Mode M>            # static_assert(std::is_floating_point_v<T>); instantiated for float; Mode ∈ { Madgwick, Mahony }
class AhrsFilter:
    math::Quaternion<T>  q              # body->earth orientation, kept unit-norm (item 18)
    T                    Ts             # sample period
    T                    beta           # Madgwick gradient gain
    T                    Kp, Ki         # Mahony proportional / integral gains
    math::Vector3<T>     integralFb     # Mahony gyro-bias estimate (Ki path)
```

## Interface

```
AhrsFilter(T gain, T Ts)                                # beta (Madgwick) or Kp (Mahony)
void UpdateImu(Vector3 gyro, Vector3 accel)            # 6-DOF, no heading reference
void UpdateMarg(Vector3 gyro, Vector3 accel, Vector3 mag)  # 9-DOF, adds heading
Quaternion Orientation() const
Vector3    Euler() const                               # roll/pitch/yaw convenience
void       Reset()                                     # q = identity, bias = 0
```

## Algorithm (pseudocode)

```
function UpdateImu(gyro, accel):             # OPTIMIZE_FOR_SPEED
    if Norm(accel) == 0: return              # free-fall ⇒ gyro-only, skip correction
    a = Normalize(accel)

    if Mode == Mahony:
        v      = GravityFromQuaternion(q)     # 3rd row of R(q): predicted gravity direction
        e      = CrossProduct(a, v)           # correction error (accel vs. predicted)
        integralFb += Ki * e * Ts             # integrate ⇒ estimate gyro bias
        omega   = gyro + Kp * e + integralFb  # bias-compensated, error-corrected rate
        qDot    = 0.5 * (q ⊗ Quaternion(0, omega))

    else:   # Madgwick
        f        = ObjectiveGravity(q, a)     # 3-vector: R(q)^T*g_ref - a
        gradient = Normalize(JacobianGravity(q)^T * f)   # steepest descent, 4-vector
        qDot     = 0.5 * (q ⊗ Quaternion(0, gyro)) - beta * gradient

    q = Normalize(q + qDot * Ts)              # integrate + re-normalize

function UpdateMarg(gyro, accel, mag):
    # as UpdateImu, plus a magnetometer term that constrains yaw:
    #   reference earth field b = [bx, 0, bz] from the current tilt,
    #   append the mag objective/error to f (Madgwick) or e (Mahony).
    ...
```

## Complexity & memory

- Time: `O(1)` per update — a fixed few dozen multiply-adds, one inverse-sqrt normalization.
- Memory: `O(1)` — one quaternion (4 floats) plus a 3-float bias term; no buffers, no heap.

## Numerical / embedded notes

- Normalize `accel`/`mag` each step and **skip** the correction when a norm is ≈ 0 (free-fall, or a
  magnetically disturbed sample) so a bad measurement cannot corrupt the attitude.
- Quaternion state is **singularity-free** (no gimbal lock); only the optional Euler output has it.
- `beta` scales with expected gyro measurement error; Mahony's `Kp`/`Ki` set the correction
  bandwidth and the bias-learning rate respectively.
- Re-normalize `q` after every integration to keep it on the unit 3-sphere.
- Reuses `math::Quaternion` (item 18) and `math::Geometry3D` (`CrossProduct`, `VectorNorm`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/active/AhrsMadgwickMahony.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `UpdateImu`/`UpdateMarg`, and
  `extern template class AhrsFilter<float, Madgwick>;` / `extern template class AhrsFilter<float, Mahony>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/active/AhrsMadgwickMahony.cpp` →
  `template class AhrsFilter<float, Madgwick>; template class AhrsFilter<float, Mahony>;`
- Test: `numerical/filters/active/test/TestAhrsMadgwickMahony.cpp`
- Doc: `doc/filters/active/AhrsMadgwickMahony.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestAhrsMadgwickMahony.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
