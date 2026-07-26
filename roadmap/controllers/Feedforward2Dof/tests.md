# Feedforward / 2-DOF Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class MockFeedforward : public Feedforward<float>:
    MOCK_METHOD(float, Evaluate, (float), (const, override))

class MockFeedbackLaw : public FeedbackLaw<float>:
    MOCK_METHOD(float, Process, (float), (override))

class TestFeedforward2Dof : public ::testing::Test:
    StrictMock<MockFeedforward> ff
    StrictMock<MockFeedbackLaw> fb
    Saturation<float> clamp{ -1.0f, 1.0f }
    Feedforward2Dof<float> controller{ ff, fb, clamp }
# each case below is a TEST_F(TestFeedforward2Dof, <name>)
```

## Test cases (Arrange / Act / Assert)

```
sums_feedforward_and_feedback:
    Arrange: EXPECT ff.Evaluate(r) -> 0.3, fb.Process(r-y) -> 0.2
    Act:     Compute(r=0.5, y=0.1)
    Assert:  result ≈ 0.5

passes_correct_error_to_feedback:
    Arrange: r = 0.4, y = 0.1
    Assert:  fb.Process called with 0.3 (= r - y)

passes_reference_to_feedforward:
    Arrange: r = 0.6
    Assert:  ff.Evaluate called with 0.6 (the reference, not the error)

zero_feedforward_reduces_to_feedback:
    Arrange: ff.Evaluate -> 0
    Assert:  output equals the feedback term alone

output_is_clamped:
    Arrange: ff -> 0.9, fb -> 0.9 (sum 1.8), clamp [-1,1]
    Assert:  result == 1.0

perfect_feedforward_zero_error:
    Arrange: y == r so error 0, fb.Process(0) -> 0, ff -> u*
    Assert:  output == u* (feedforward carries the command)

reset_delegates_to_feedback:
    Arrange: Reset()
    Assert:  feedback state cleared (next error processed fresh)

negative_reference_handled:
    Arrange: r = -0.3, ff -> -0.2, fb -> -0.1
    Assert:  result ≈ -0.3
```

## Reference vectors

- `u = Kff·r + fb(r−y)`; with `Kff = 0` the 2-DOF law collapses to pure feedback.
- With an exact inverse-plant feedforward and `y = r`, feedback contributes zero.

## Edge cases

- Saturation acts on the summed command (not on each term separately).
- StrictMock verifies feedforward sees `r` while feedback sees `r − y` (no argument mix-up).
