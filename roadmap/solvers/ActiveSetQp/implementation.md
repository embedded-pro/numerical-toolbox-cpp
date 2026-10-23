# Dense Convex QP (Goldfarb–Idnani dual active set) — Implementation Pseudocode

> Roadmap ref: #65 (Tier 4) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

Exact solver for small, dense, strictly convex quadratic programs

```text
minimize ½·xᵀHx + gᵀx   subject to   E·x = e,   C·x ≥ c        (H symmetric positive definite)
```

in a bounded number of iterations, with no heap. The dual active-set method starts from the
unconstrained minimum and adds the most violated constraint each iteration, so it needs no feasible
starting point and terminates finitely. Consumers: exact cable-tension distribution and task-priority /
whole-body control (robotics-toolbox-cpp), control allocation with actuator limits, `Mpc` with input
constraints (whose current box handling is a fixed-count projected coordinate descent), minimum-snap
trajectories.

## Data structures

```cpp
template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
struct QuadraticProgram:                       # static_assert(std::is_floating_point_v<T>)
    math::SquareMatrix<T, Variables>           hessian            # H, SPD
    math::Vector<T, Variables>                 gradient           # g
    math::Matrix<T, Equalities, Variables>     equalityMatrix     # E
    math::Vector<T, Equalities>                equalityVector     # e
    math::Matrix<T, Inequalities, Variables>   inequalityMatrix   # C   (rows: C_i·x ≥ c_i)
    math::Vector<T, Inequalities>              inequalityVector   # c
    # bounds lo ≤ x ≤ hi are two inequality rows per bounded variable (+e_i ≥ lo_i, −e_i ≥ −hi_i)

enum class QpStatus : uint8_t { Optimal, Infeasible, NotPositiveDefinite, IterationLimit }

template<typename T, std::size_t Variables, std::size_t Constraints>   # Constraints = Equalities + Inequalities
struct QpResult:
    math::Vector<T, Variables>   x
    math::Vector<T, Constraints> multipliers   # equalities first; inequality multipliers ≥ 0, zero if inactive
    std::array<bool, Constraints> active
    std::size_t                  iterations
    QpStatus                     status

template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>   # instantiated for float
class ActiveSetQp:
    std::size_t maxIterations                  # default 2·(Equalities + Inequalities) + Variables
    T           feasibilityTolerance           # relative to the constraint-row scale
```

`math::Matrix` has no zero-size form, so `Equalities = 0` is a partial specialization that omits the
equality block (and `Inequalities = 0` likewise); all other code paths are shared.

## Interface

```cpp
explicit ActiveSetQp(std::size_t maxIterations = default, T feasibilityTolerance = 1e-6)
QpResult<T, Variables, Equalities + Inequalities> Solve(const QuadraticProgram<...>& qp) const   # hot path
```

## Algorithm (pseudocode)

Goldfarb & Idnani (1983) in the numerically stable form of Powell (1985): the active constraint normals
`N` are tracked through `J = L⁻ᵀ·Q` (`Q` orthogonal) and an upper-triangular `R` with `Jᵀ·N = [R; 0]`;
adding or dropping a constraint updates `J` and `R` with Givens rotations.

```text
function Solve(qp):                            # OPTIMIZE_FOR_SPEED
    L = Cholesky(H);  if fails: return { status = NotPositiveDefinite }
    J = L⁻ᵀ                                    # triangular inverse, O(n³) once
    x = −H⁻¹·g;  q = 0 (active count);  u = 0;  active = none
    pending = all equalities                   # equalities are added first, in order
    for iter in 0..maxIterations−1:
        # Step 1 — choose the constraint to add
        if pending not empty:   p = next equality;  s = E_p·x − e_p
        else:
            s, p = min over inactive inequalities i of (C_i·x − c_i)         # most violated (raw residual)
            if s ≥ −feasibilityTolerance·‖C_p‖: return { x, u, active, iter, Optimal }
        n⁺ = constraint normal of p (sign-flipped for an equality with s > 0, so s ≤ 0)
        loop:                                  # Step 2 — may drop several constraints before adding p
            d = Jᵀ·n⁺
            z = J₂·d₂                          # primal direction (columns q..n−1 of J)
            r = R⁻¹·d₁                         # dual direction for the active set (back substitution)
            t₁ = min over active INEQUALITIES j with r_j > 0 of u_j / r_j  (∞ if none; k = argmin)
            t₂ = ‖z‖ > ε ? −s / (zᵀ·n⁺) : ∞
            t  = min(t₁, t₂)
            if t == ∞: return { x, u, active, iter, Infeasible }        # p cannot be satisfied
            u_active = u_active − t·r;  u_p = u_p + t
            if t₂ < ∞: x = x + t·z;  s = s + t·(zᵀ·n⁺)
            if t == t₂:                        # full step: p becomes active
                add p: rotate d₂ into its first entry with Givens on J₂'s columns; append column to R; q += 1
                break
            drop k: delete column k of R, restore upper-triangular with Givens (applied to J); q −= 1
        if p was the last pending equality: pending = empty
    return { x, u, active, maxIterations, IterationLimit }
```

## Complexity & memory

- Setup: one Cholesky and one triangular inverse, `O(n³)` with `n = Variables`.
- Per iteration: `O(n²)` for `d`, `z`, `r` and the Givens updates; at most `maxIterations` iterations
  (finite termination is guaranteed in exact arithmetic; the cap guards rounding-induced cycling).
- Memory: `J` (`n×n`), `R` (`n×n`), multipliers and flags; stack only, no heap.

## Numerical / embedded notes

- **Strict convexity is required** (`H` SPD): a pure LP or a semidefinite `H` fails in Cholesky
  (`NotPositiveDefinite`); add a small `ε·I` regularization when an LP-like problem must be solved this way.
  robotics-toolbox-cpp's TOPP-RA keeps its own two-variable LP for that reason.
- Dual feasibility (multipliers of active inequalities ≥ 0) holds at every iteration; primal feasibility is
  reached at termination — an early stop returns a super-optimal, slightly infeasible `x`.
- Scale constraint rows to comparable norms: the constraint to add is the largest *raw* violation, and
  only the acceptance test is relative to `‖C_p‖`.
- Degenerate (linearly dependent) active constraints show up as `‖z‖ ≈ 0` with a blocking `t₁` — the
  drop branch handles them; truly contradictory constraints give `t = ∞` ⇒ `Infeasible`.
- Warm start (re-using the previous active set) is a possible later extension for MPC loops.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/solvers/ActiveSetQp.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class ActiveSetQp<float, 2, 1, 2>;` / `<float, 2, 0, 4>` / `<float, 4, 2, 8>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/ActiveSetQp.cpp` → the same instantiations.
- Test: `numerical/solvers/test/TestActiveSetQp.cpp`
- Doc: `doc/solvers/ActiveSetQp.md` (per `doc/TEMPLATE.md`) + row in `doc/solvers/README.md`
- CMake: `.hpp` → `target_sources(numerical.solver PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.solver ...)`; `TestActiveSetQp.cpp` → `numerical.solvers_test`.
- Follow-up: `Mpc` input constraints can switch from projected coordinate descent to this solver.
- Depends on: `math::CholeskyDecomposition`, `math::TriangularSolve`, `math::GivensRotation`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
