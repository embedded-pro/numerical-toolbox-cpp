# Madgwick / Mahony AHRS — Overview

## What it is
An **Attitude and Heading Reference System**: a filter that fuses a gyroscope with an accelerometer
(and optionally a magnetometer) into a single **quaternion** describing the body's orientation.
Two closely-related formulations share this spec — Madgwick's **gradient-descent** correction and
Mahony's **passive complementary filter on SO(3)** — both correcting a gyro-integrated quaternion
with a vector-observation error.

## Why it matters (embedded)
It is the de-facto attitude estimator for drones, robots, AR headsets and wearables: cheaper and
more robust than a full quaternion EKF, with a fixed `O(1)` cost and no matrices to invert. It
turns three cheap MEMS sensors into a stable, drift-free 3-D orientation at loop rate.

## How it works (intuition)
The gyro tells you how orientation is *changing*; integrating it is accurate short-term but drifts.
The accelerometer gives an absolute **gravity** reference and the magnetometer an absolute **north**
reference — noisy, but drift-free. Each step predicts the new quaternion from the gyro, then nudges
it so the *predicted* gravity/north directions line up with the *measured* ones. Madgwick expresses
that nudge as one step of gradient descent on the alignment error; Mahony expresses it as a
proportional-integral feedback whose integral term also **learns the gyro bias**. Working in
quaternions keeps the whole thing free of gimbal lock.

## Key parameters
- **β (Madgwick gain)** — trades gyro trust against accel/mag correction (∝ gyro error).
- **Kp, Ki (Mahony gains)** — correction bandwidth and gyro-bias learning rate.
- **Ts** — sample period.
- **6-DOF vs 9-DOF** — IMU (gravity only, yaw free) vs MARG (adds magnetometer heading).

## Reference
S. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays,"
2010; R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special
Orthogonal Group," *IEEE Trans. Automatic Control*, 53(5), 2008.

## See also
`ComplementaryFilter` (the scalar 1-D ancestor), `Quaternion` (item 18, the state type),
`ExtendedKalmanFilter` (the heavier probabilistic alternative).
