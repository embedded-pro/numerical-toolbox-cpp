# Pose Inverse Kinematics (Damped Least Squares) — Implementation Pseudocode

> Roadmap ref: #M13 (Tier 3) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct PoseIkConfig:
    T           damping      # λ (Levenberg-Marquardt)
    T           tolerance    # 6-vector error norm to declare success
    std::size_t maxIterations

template<typename T, std::size_t NumLinks>
struct PoseIkResult:
    Vector<T, NumLinks> q
    T                   finalError
    std::size_t         iterations
    bool                converged

template<typename T, std::size_t NumLinks>
class PoseInverseKinematics:
    DenavitHartenberg<T, NumLinks>  fk
    SpatialJacobian<T, NumLinks>    jac
    PoseIkConfig<T>                 config
```

## Interface

```
PoseInverseKinematics(model, PoseIkConfig<T> cfg = {})
PoseIkResult<T, N>  Solve(SE3<T> target, JointVector q0)        # hot path
Vector<T, 6>        PoseError(SE3<T> target, SE3<T> current)     # [Δp; Δω]
```

## Algorithm (pseudocode)

```
function PoseError(target, current):
    e_p   = target.p - current.p                # position error
    R_err = target.R * Transpose(current.R)     # relative rotation
    e_ω   = AxisAngle(R_err)                      # log map → rotation vector
    #  equivalently 2·(vector part of quaternion(R_err)); reuse item 18
    return concat(e_p, e_ω)                       # 6-vector

function Solve(target, q0):                     # OPTIMIZE_FOR_SPEED
    q = q0
    for iter in 0..maxIterations-1:
        T = fk.Forward(q)
        e = PoseError(target, T)                # 6-vector
        if norm(e) < tolerance: return { q, norm(e), iter, true }
        J = jac.Compute(q)                      # 6×N
        # damped least squares: Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
        A = J * Transpose(J) + λ² * I₆          # 6×6 SPD
        y = GaussianElimination(A, e)            # solve A y = e  (reuse solver)
        q = q + Transpose(J) * y
    return { q, norm(e), maxIterations, false }
```

## Complexity & memory

- Per iteration: `O(6²N)` to form `J Jᵀ` plus `O(6³)` for the 6×6 solve.
- Total: `O(iters · (6²N + 6³))`; iteration count depends on `λ` and start pose.
- Memory: one `6×N` Jacobian and a `6×6` system; no heap, no dynamic sizing.

## Numerical / embedded notes

- **Damping `λ`** trades accuracy for stability: it keeps `(J Jᵀ + λ²I)` invertible *through*
  singularities (Nakamura's singularity-robust inverse) at the cost of a small steady-state error.
- Orientation error **must** use the log/quaternion form — naive Euler-angle subtraction wraps and
  stalls; reuse `Quaternion` (item 18) and fix the double-cover sign so the arm takes the short way.
- Solve `A y = e` with `GaussianElimination`; never form `(J Jᵀ)⁻¹` explicitly.
- Seed `q0` from the previous control cycle for warm-started, few-iteration convergence.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/PoseInverseKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class PoseInverseKinematics<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/PoseInverseKinematics.cpp` → `template class PoseInverseKinematics<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestPoseInverseKinematics.cpp`
- Doc: `doc/kinematics/PoseInverseKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestPoseInverseKinematics.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
