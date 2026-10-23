# Dense Convex QP (Goldfarb–Idnani) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestActiveSetQp : public ::testing::Test:
    # least-distance problems: H = I, g = −x_ref  ⇒  minimize ½‖x − x_ref‖²
    ActiveSetQp<float, 2, 1, 2> withEquality
    ActiveSetQp<float, 2, 0, 4> boxOnly
# each case below is a TEST_F(TestActiveSetQp, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
inactive_constraints_return_the_unconstrained_minimum:
    Arrange: x_ref = (1, 1); box −5 ≤ x ≤ 5
    Assert:  x ≈ (1, 1), status Optimal, all multipliers 0, no constraint active
equality_constraint_is_met_with_its_multiplier:
    Arrange: x_ref = (0, 0); x₁ + x₂ = 1; inequalities x ≥ −5 (inactive)
    Assert:  x ≈ (0.5, 0.5), equality multiplier ≈ 0.5
violated_inequality_becomes_active:
    Arrange: boxOnly-style rows: −x₁ − x₂ ≥ −1 (x₁ + x₂ ≤ 1), x_ref = (1, 1)
    Assert:  x ≈ (0.5, 0.5), that multiplier ≈ 0.5, active
box_bounds_clip_to_the_nearest_corner:
    Arrange: x_ref = (2, −1); 0 ≤ x ≤ 1 (four rows)
    Assert:  x ≈ (1, 0), multipliers of x₁ ≤ 1 and x₂ ≥ 0 ≈ 1, others 0
blocking_constraint_is_dropped_on_the_way:
    Arrange: x_ref = (0, 0); rows x₂ ≥ 1 (raw violation 1) and 0.1·x₁ + 0.5·x₂ ≥ 0.9 (raw violation 0.9)
    Assert:  x ≈ (0.346154, 1.730769), only the second row active, its multiplier ≈ 3.461538
             (x₂ ≥ 1 is added first, its multiplier turns negative when the second row joins, and it is dropped)
contradictory_constraints_are_infeasible:
    Arrange: x₁ ≥ 1 and x₁ ≤ 0
    Assert:  status Infeasible
indefinite_hessian_is_rejected:
    Arrange: H = diag(1, −1)
    Assert:  status NotPositiveDefinite
redundant_constraints_do_not_break_the_solve:
    Arrange: x₁ + x₂ ≤ 1 stated twice, x_ref = (1, 1)
    Assert:  x ≈ (0.5, 0.5), Optimal; the two multipliers sum to 0.5
```

## Reference vectors

- `min ½‖x − (1,1)‖²`, `x₁ + x₂ ≤ 1` ⇒ `x = (0.5, 0.5)`, multiplier `0.5` (KKT: `x − x_ref = −μ·(1, 1)`).
- `min ½‖x‖²`, `x₂ ≥ 1`, `0.1x₁ + 0.5x₂ ≥ 0.9` ⇒ projection onto the second row,
  `x = 0.9·(0.1, 0.5)/0.26 = (0.346154, 1.730769)`, multiplier `0.9/0.26 = 3.461538`; `x₂ ≥ 1` holds there
  (inactive). Projecting onto `x₂ = 1` first gives `(0, 1)`, which violates the second row (`0.5 < 0.9`).
- `min ½‖x − (2,−1)‖²`, `0 ≤ x ≤ 1` ⇒ `(1, 0)`, multipliers `1` and `1`.

## Edge cases

- `Equalities = Variables` (fully determined) ⇒ `x = E⁻¹e` regardless of `g`.
- Iteration cap reached on a pathological cycle ⇒ `IterationLimit`, finite `x`.
- Large constraint-row scale differences ⇒ tolerance is relative, still `Optimal`.
