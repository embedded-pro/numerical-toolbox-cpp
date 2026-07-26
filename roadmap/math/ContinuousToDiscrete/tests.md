# Continuous-to-Discrete (c2d) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestContinuousToDiscrete : public ::testing::Test:
    ContinuousToDiscrete<float, 2, 1, 1> c2d
    LinearTimeInvariant<float, 2, 1, 1>  continuousSys   # known (A,B,C,D)
# each case below is a TEST_F(TestContinuousToDiscrete, <name>)
```

## Test cases (Arrange / Act / Assert)

```
integrator_zoh:
    Arrange: A=0, B=1 (pure integrator), Ts=0.1
    Assert:  Ad=1, Bd=Ts   (exact)

first_order_zoh_matches_analytic:
    Arrange: ẋ = -a x + b u
    Assert:  Ad = e^{-a·Ts},  Bd = (b/a)(1 - e^{-a·Ts})

zoh_preserves_C_and_D:
    Assert: Cd == C  and  Dd == D  for ZOH

forward_euler_formula:
    Assert: Ad = I + A·Ts,  Bd = B·Ts

backward_euler_formula:
    Assert: Ad = (I - A·Ts)⁻¹

tustin_bilinear_reference:
    Arrange: first-order system, Ts
    Assert:  Ad, Bd, Cd, Dd match the hand-derived bilinear result

tustin_preserves_stability:
    Arrange: stable continuous poles (Re < 0)
    Assert:  discrete poles inside the unit circle (|z| < 1)

small_ts_methods_converge:
    Arrange: Ts -> very small
    Assert:  ZOH, Tustin, Euler all approach the same Ad, Bd

dc_gain_preserved:
    Assert: discrete DC gain C(I-Ad)⁻¹Bd + Dd ≈ continuous -C A⁻¹B + D
```

## Reference vectors

- Integrator, `Ts = 0.1`, ZOH ⇒ `Ad = 1, Bd = 0.1`.
- `ẋ = -x + u`, `Ts = 0.5`, ZOH ⇒ `Ad = e^{-0.5} = 0.6065, Bd = 1 - e^{-0.5} = 0.3935`.

## Edge cases

- `Ts → 0` ⇒ `Ad → I`, `Bd → 0` for every method.
- Marginally stable pole at the origin (integrator) under Tustin ⇒ maps to `z = 1`.
- Large `Ts` with Forward Euler ⇒ document that poles may leave the unit disk.
- Singular `(aI−A)` / `(I−A·Ts)` ⇒ the solve reports degeneracy instead of dividing by zero.
