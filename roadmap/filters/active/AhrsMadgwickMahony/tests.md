# Madgwick / Mahony AHRS — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestAhrsFilter : public ::testing::Test:
    AhrsFilter<float, Madgwick> madgwick{ 0.1f, 0.01f }   # beta, Ts = 10 ms
    AhrsFilter<float, Mahony>   mahony{ 0.5f, 0.01f }     # Kp,   Ts = 10 ms
    static constexpr float g = 9.81f
# each case below is a TEST_F(TestAhrsFilter, <name>)
```

## Test cases (Arrange / Act / Assert)

```
level_static_converges_to_identity:
    Arrange: gyro = 0, accel = (0, 0, g) held
    Act:     run UpdateImu many times
    Assert:  Orientation() ≈ identity quaternion (roll=pitch=0)

tilt_recovered_from_gravity:
    Arrange: accel rotated by known roll = 30 deg, gyro = 0
    Assert:  Euler().roll ≈ 30 deg  (pitch ≈ 0, yaw free)

pure_gyro_integrates_rotation:
    Arrange: constant gyro about z, accel = (0,0,g)
    Assert:  yaw advances ≈ omega_z * Ts * n (heading unconstrained by accel)

quaternion_stays_unit_norm:
    Arrange: feed noisy gyro+accel for many steps
    Assert:  |Orientation()| ≈ 1 throughout (no drift off the sphere)

zero_accel_skips_correction:
    Arrange: accel = (0,0,0) (free-fall), constant gyro
    Assert:  update reduces to pure gyro integration, no NaN

marg_constrains_yaw:
    Arrange: UpdateMarg with mag pointing north, static
    Assert:  yaw converges to the magnetometer heading

mahony_estimates_gyro_bias:
    Arrange: true gyro = 0 but measured gyro has constant bias, accel = (0,0,g)
    Assert:  integralFb -> bias, attitude stays level

madgwick_and_mahony_agree_static:
    Arrange: same static input to both filters
    Assert:  orientations agree within tolerance

reset_restores_identity:
    Arrange: perturb, then Reset()
    Assert:  q == identity, bias == 0
```

## Reference vectors

- Static accel `(0,0,g)` ⇒ identity quaternion (`w=1`, vector part `0`).
- Roll/pitch analytically fixed by the gravity direction; yaw fixed by the mag heading (MARG).

## Edge cases

- `Norm(accel)=0` and `Norm(mag)=0` ⇒ correction skipped, no divide-by-zero.
- Large `Ts` (low update rate) ⇒ still stable, bounded error.
- 180-deg / near-antipodal orientation ⇒ no gimbal lock (quaternion), continuous convergence.
- Sustained gyro bias ⇒ Mahony integral term drives steady-state attitude error to ~0.
