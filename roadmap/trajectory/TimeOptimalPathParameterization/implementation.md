# Time-Optimal Path Parameterization (TOPP) — Implementation Pseudocode

> Roadmap ref: #M27 (Tier 5) · Target: `numerical/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>        # static_assert(std::is_floating_point_v<T>); instantiated for float
interface PathGeometry:                    # DI — the fixed geometric path q(s), s ∈ [0,1]
    math::Vector<T, N> Position(T s)
    math::Vector<T, N> FirstDerivative(T s)    # q'(s)
    math::Vector<T, N> SecondDerivative(T s)   # q''(s)

template<typename T, std::size_t N>        # static_assert(std::is_floating_point_v<T>); instantiated for float
interface JointLimits:                     # DI — inverse dynamics + bounds (reuse RNEA)
    void   Coefficients(T s, out a, out b, out c)   # τ = a(s)·s̈ + b(s)·ṡ² + c(s)
    Bounds TorqueBounds(), VelocityBounds()

template<typename T, std::size_t N, std::size_t Grid>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class TimeOptimalPathParameterization:
    array<T, Grid>  sGrid                  # discretized path parameter
    array<T, Grid>  xMax                   # MVC: max ṡ² per gridpoint
    array<T, Grid>  xProfile               # controllable ṡ² after reachability pass
```

## Interface

```
TimeOptimalPathParameterization(PathGeometry<T>&, JointLimits<T>&)
bool         Parameterize()                # build ṡ²(s); false if infeasible
Optional<TrajectoryState<T>> Sample(T t)   # hot path, after parameterization
T            TotalTime()
```

## Algorithm (pseudocode)

```
# Per gridpoint the joint torque law is affine in (u = s̈, x = ṡ²):
#   τ_j(s) = a_j(s)·u + b_j(s)·x + c_j(s)   ∈ [τ_min, τ_max]

function maxVelocityCurve(s):              # MVC — tightest ṡ² still feasible
    x_v = min_j ( velLimit_j / |q'_j(s)| )^2          # velocity bound
    x_a = largest x where some u keeps every τ_j in bounds   # accel/torque bound
    return min(x_v, x_a)

function Parameterize():                    # TOPP-RA reachability analysis
    for i in 0..Grid-1: xMax[i] = maxVelocityCurve(sGrid[i])
    # backward pass: controllable set ending at rest (x_N = 0)
    for i = Grid-1 .. 0:
        [uLo, uHi] = admissibleAccel(sGrid[i], xProfile[i])   # 1-D LP over joints
        xProfile[i] = min(xMax[i], propagateBack(xProfile[i+1], uLo, uHi))
    # forward pass: reachable set starting from rest (x_0 = 0), clipped to controllable
    for i = 0 .. Grid-1:
        xProfile[i] = min(xProfile[i], propagateForward(xProfile[i-1]))
    return all(xProfile >= 0)

function Sample(t):                         # OPTIMIZE_FOR_SPEED
    # integrate dt = ds / sqrt(x(s)) offline into a time->s table, then look up
    s   = timeToS(t)
    sd  = sqrt(interp(xProfile, s))         # ṡ
    sdd = admissibleAccelMid(s)             # s̈
    return jointState(path, s, sd, sdd)
```

## Complexity & memory

- Parameterization: `O(Grid · N)` — each gridpoint solves a tiny 1-D LP over `N` joints.
- Two linear passes (backward controllable + forward reachable), no global optimization loop.
- Memory: `O(Grid)` bounded arrays (`sGrid`, `xMax`, `xProfile`); planning-time, stack/static only.

## Numerical / embedded notes

- This is a **planning-time** computation, not an ISR path — run offline, then stream `Sample`.
- Reuse **RNEA** (item exists) to evaluate the `a(s), b(s), c(s)` inverse-dynamics coefficients per
  gridpoint; inject it behind `JointLimits` so the planner stays dynamics-agnostic (DIP).
- Grid density trades accuracy for memory/time; `Grid` is a compile-time bound (`std::array`).
- Guard **MVC singularities** (a `q'_j(s)=0` "zero-inertia" direction) — clamp, don't divide by zero.
- TOPP-RA's reachability formulation is numerically robust where Bobrow's switch-point search stalls.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/trajectory/TimeOptimalPathParameterization.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class TimeOptimalPathParameterization<float, N, Grid>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/trajectory/TimeOptimalPathParameterization.cpp` →
  `template class TimeOptimalPathParameterization<float, N, Grid>;`
- Test: `numerical/trajectory/test/TestTimeOptimalPathParameterization.cpp`
- Doc: `doc/trajectory/TimeOptimalPathParameterization.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestTimeOptimalPathParameterization.cpp` → the `_test` target.
- New module: create `numerical/trajectory/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/CMakeLists.txt`, and add a `doc/trajectory/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
