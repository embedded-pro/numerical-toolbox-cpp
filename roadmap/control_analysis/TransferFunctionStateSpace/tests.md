# Transfer-Function ↔ State-Space — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestTransferFunctionStateSpace : public ::testing::Test:
    # H(s) = (s + 2) / (s² + 3s + 2)   -> den=[1,3,2], num=[0,1,2]
    TransferFunction<float,2> tf = MakeSecondOrder()
    TransferFunctionStateSpace<float,2> convert
# each case below is a TEST_F(TestTransferFunctionStateSpace, <name>)
```

## Test cases (Arrange / Act / Assert)

```
controllable_form_matches_hand_value:
    Act:     sys = ToControllableCanonical(tf)
    Assert:  A == [[0, 1], [-2, -3]],  B == [0; 1],  C == [2, 1],  D == 0

companion_bottom_row_is_negated_denominator:
    Assert:  A(1,·) == [-a2, -a1] == [-2, -3]

observable_form_is_dual_of_controllable:
    Arrange: ccf = ToControllableCanonical(tf)
    Act:     ocf = ToObservableCanonical(tf)
    Assert:  ocf.A == ccf.Aᵀ and ocf.B == ccf.Cᵀ and ocf.C == ccf.Bᵀ

state_space_to_tf_recovers_denominator:
    Arrange: sys = ToControllableCanonical(tf)
    Act:     tf2 = ToTransferFunction(sys)
    Assert:  tf2.denominator ≈ [1, 3, 2]   # characteristic polynomial of A

round_trip_tf_ss_tf_is_identity:
    Act:     tf2 = ToTransferFunction(ToControllableCanonical(tf))
    Assert:  tf2.numerator ≈ tf.numerator and tf2.denominator ≈ tf.denominator

feedthrough_extracted_for_proper_tf:
    Arrange: proper H(s) = (2s² + 3s + 4)/(s² + 3s + 2)   # deg(num)==deg(den)
    Act:     sys = ToControllableCanonical(H)
    Assert:  D == 2  and remaining C from strictly-proper remainder

strictly_proper_has_zero_feedthrough:
    Assert:  ToControllableCanonical(tf).D == 0

denominator_normalized_to_monic:
    Arrange: non-monic den = [2, 6, 4]
    Assert:  realisation identical to monic den = [1, 3, 2] (scaled numerator too)

both_forms_share_impulse_response:
    Arrange: ccf and ocf of tf
    Assert:  identical first K impulse-response samples (same I/O behaviour)
```

## Reference vectors

- `H(s) = (s+2)/(s²+3s+2)` ⇒ CCF `A=[[0,1],[-2,-3]], B=[0;1], C=[2,1], D=0`.
- Observable form is the transpose quadruple of the above.
- Round trip `TF → SS → TF` must return `num=[0,1,2]`, `den=[1,3,2]` within tolerance.

## Edge cases

- First order (`n = 1`) — companion collapses to a scalar `A`.
- Proper transfer function (`deg(num) == deg(den)`) — nonzero `D`.
- Pole-zero cancellation, e.g. `(s+1)/(s²+2s+1)` — non-minimal realisation; conversion still valid.
- Non-monic / near-zero leading denominator coefficient — must normalise or guard.
