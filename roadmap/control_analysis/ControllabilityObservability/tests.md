# Controllability / Observability — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestControllabilityObservability : public ::testing::Test:
    # 2-state SISO plant in controllable canonical form (guaranteed controllable & observable)
    #   A = [[0, 1], [-2, -3]],  B = [0; 1],  C = [1, 0],  D = 0
    math::LinearTimeInvariant<float,2,1,1> plant = MakeCanonical()
    ControllabilityObservability<float,2,1,1> analysis
# each case below is a TEST_F(TestControllabilityObservability, <name>)
```

## Test cases (Arrange / Act / Assert)

```
controllability_matrix_matches_hand_value:
    Arrange: canonical plant
    Act:     Wc = ControllabilityMatrix(plant)
    Assert:  Wc == [[0, 1], [1, -3]]           # [B  AB]

observability_matrix_matches_hand_value:
    Act:     Wo = ObservabilityMatrix(plant)
    Assert:  Wo == [[1, 0], [0, 1]]            # [C; CA]

controllable_pair_has_full_rank:
    Assert:  IsControllable(plant) == true and Rank(Wc) == 2

uncontrollable_pair_detected:
    Arrange: diagonal A = diag(0.5, 0.5) with B = [1; 0]   # second mode unreachable
    Assert:  IsControllable == false, Rank(Wc) == 1

observable_pair_has_full_rank:
    Assert:  IsObservable(plant) == true

unobservable_pair_detected:
    Arrange: diagonal A = diag(0.5, 0.5) with C = [1, 0]   # second mode invisible
    Assert:  IsObservable == false

duality_ctrb_equals_obsv_of_transpose:
    Arrange: plant and its dual (Aᵀ, Cᵀ, Bᵀ)
    Assert:  ControllabilityMatrix(dual) == Transpose(ObservabilityMatrix(plant))

gramian_solves_discrete_lyapunov:
    Arrange: Schur-stable A = [[0, 1], [-0.2, -0.3]]
    Act:     Wc = ControllabilityGramian(plant)
    Assert:  ‖A·Wc·Aᵀ − Wc + B·Bᵀ‖ < tol       # residual of Lyapunov equation

gramian_is_symmetric_positive_definite:
    Arrange: stable + controllable plant
    Assert:  Wc == Wcᵀ and all eigenvalues > 0
```

## Reference vectors

- Canonical `A=[[0,1],[-2,-3]], B=[0;1]` ⇒ `Wc = [B AB] = [[0,1],[1,-3]]`, `det = −1 ≠ 0` ⇒ full rank.
- Diagonal `A=diag(0.5,0.5)` with a decoupled mode ⇒ deterministic rank deficiency (rank 1).
- Duality golden: `Ctrb(Aᵀ,Cᵀ) == Obsv(A,C)ᵀ`.

## Edge cases

- Single state (`n = 1`) — matrices collapse to scalars; rank is `0` or `1`.
- Marginally controllable (near-linearly-dependent Krylov columns) — verify tolerance decides rank.
- Unstable `A` (spectral radius ≥ 1) — Gramian sum diverges; the solver must report/guard, not silently return garbage.
- Repeated eigenvalues with a shared input direction — classic loss of controllability.
