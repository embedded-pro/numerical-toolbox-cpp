# Cable Tension Distribution — Implementation Pseudocode

> Roadmap ref: #M25 (Tier 4) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t NumCables, std::size_t WrenchDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class CableTensionDistribution:                                 # WrenchDim = 6 (or 3 planar)
    const kinematics::SpatialJacobian<T, NumCables>& structure  # J → A = −Jᵀ (WrenchDim×NumCables), injected (#M8)
    controllers::Mpc<T, NumCables, WrenchDim> qpSolver          # reused bounded-QP machinery
    T tMin        # minimum cable tension (> 0: cables never go slack)
    T tMax        # maximum cable tension (actuator / cable limit)
```

## Interface

```
CableTensionDistribution(const SpatialJacobian& structure, T tMin, T tMax)

# returns nullopt when the pose is outside the wrench-feasible workspace:
std::optional<Vector<T,NumCables>>
    Distribute(const WrenchVector& wDesired, const StateVector& pose)   # hot path
```

## Algorithm (pseudocode)

```
function Distribute(wDesired, pose):        # OPTIMIZE_FOR_SPEED
    A = -transpose(structure.Compute(pose))       # WrenchDim×NumCables structure matrix
    # pull-only, bounded tensions that realise the wrench, kept away from the limits:
    #   minimise ½‖t − tMid‖²                       (closest-to-centre ⇒ max disturbance margin)
    #   s.t.     A·t = wDesired                      (exact wrench, equality)
    #            tMin ≤ t ≤ tMax                      (cables pull only, t > 0)
    tMid   = 0.5*(tMin + tMax) * Ones(NumCables)
    result = qpSolver.Solve(H = Identity(NumCables), gradient = -tMid,
                            equality = { A, wDesired },
                            lower = tMin, upper = tMax)
    if not result.feasible:
        return nullopt                            # pose outside wrench-feasible workspace
    return result.tension
```

## Complexity & memory

- Time: bounded active-set / interior-point QP with `NumCables` variables and `WrenchDim` equalities:
  `O(NumCables³)` worst case; small (`NumCables ≤ 8`).
- Memory: `O(NumCables²)` working matrices; all bounded/static, no heap.

## Numerical / embedded notes

- **Cables pull only** (`t ≥ tMin > 0`) — the defining constraint. A rigid-robot statics solve can
  return compression, which is physically impossible here, so the bounded QP is mandatory.
- Redundancy (`NumCables > WrenchDim`) leaves a null space; **centring** tensions at `tMid` keeps
  them away from slack (`tMin`) and snap (`tMax`), maximising the wrench-disturbance margin (Pott).
- **Infeasible ⇒ `nullopt`, never an exception** — the caller treats it as a workspace-boundary
  event; matches the no-exception embedded convention.
- Keep `tMin > 0` so cables stay taut (avoids backlash / control loss); size `tMax` to the winch.
- A 2-norm (or Δt-regularised) objective gives **continuous** tensions between cycles — no chatter
  when the desired wrench moves smoothly.
- Structure matrix `A = −Jᵀ` reuses the spatial Jacobian (#M8); the QP reuses the `Mpc` solver.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/CableTensionDistribution.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Distribute`, and
  `extern template class CableTensionDistribution<float, NumCables, WrenchDim>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/CableTensionDistribution.cpp` →
  `template class CableTensionDistribution<float, NumCables, WrenchDim>;`
- Test: `numerical/controllers/manipulator/test/TestCableTensionDistribution.cpp`
- Doc: `doc/controllers/manipulator/CableTensionDistribution.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestCableTensionDistribution.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
