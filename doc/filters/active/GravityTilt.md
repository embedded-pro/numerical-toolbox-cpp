# Gravity Tilt and Trust

## Overview & Motivation

At rest an accelerometer measures only gravity. The direction of the measured vector then gives the tilt of the
body, with no drift. When the body accelerates, the sensor also measures that acceleration, and the tilt is wrong.
An attitude filter fuses this drift-free but disturbed angle with the gyroscope, which is smooth but drifts.

This algorithm gives the tilt in one plane and a trust weight from 0 to 1. The trust tells the filter how far to
believe the angle at this sample: fully when the sensor sees gravity alone, not at all when the body clearly
accelerates.

## Mathematical Theory

### Tilt in one plane

Let $a_\parallel$ and $a_\perp$ be the measured components in the tilt plane, with $a_\perp$ along gravity at zero
tilt. Then

$$\theta = \operatorname{atan2}(a_\parallel,\ a_\perp).$$

The two-argument arctangent keeps the quadrant, so the angle covers the whole circle. The caller picks the axes and
their signs to match its body frame.

### Trust from the magnitude

With the third component $a_\ell$, the measured magnitude is

$$|a| = \sqrt{a_\parallel^2 + a_\perp^2 + a_\ell^2}.$$

At rest $|a| = g$. Any extra acceleration moves $|a|$ away from $g$, so the relative error is a cheap test for
motion. With the band $b$, the relative error at which trust reaches zero,

$$w = \operatorname{clamp}\!\left(1 - \frac{\bigl||a| - g\bigr|}{b\,g},\ 0,\ 1\right).$$

A filter scales its accelerometer correction by $w$, for example the gain of a complementary filter or the inverse
of the measurement variance of a Kalman filter.

### Limits of the test

Acceleration at a right angle to gravity changes the magnitude only to second order. A small sideways push therefore
keeps a high trust while it bends the angle. The test catches large and vertical accelerations, not every one.

## Complexity Analysis

| Operation | Time   | Space  | Notes                                  |
|-----------|--------|--------|----------------------------------------|
| Tilt      | $O(1)$ | $O(1)$ | One arctangent                         |
| Trust     | $O(1)$ | $O(1)$ | One square root, one division, a clamp |

## Step-by-Step Walkthrough

With $g = 9.81$ m/s² and $b = 0.2$:

1. At 0.3 rad of tilt at rest, the components are $(2.899, 9.372, 0)$: the angle is 0.3 rad and the trust is 1.
2. A vertical push to $|a| = 10.79$ m/s² is 10 % above $g$: the trust is $1 - 0.1/0.2 = 0.5$.
3. A push to $|a| = 14.7$ m/s² is 50 % above $g$: the trust is 0, and the angle is not used.

## Pitfalls & Edge Cases

- **Zero vector**: free fall or a dead sensor gives magnitude 0, so the trust is 0 and the undefined angle never
  counts. The result is then an angle of 0 with trust 0.
- **Bad input**: a component that is not finite, a gravity that is not positive or a band that is not positive gives
  an angle of 0 with trust 0.
- **Axis signs**: a wrong sign on either in-plane axis mirrors the angle. Check the sign with the body at a known tilt.
- **Gravity value**: use the local gravity, or the magnitude the sensor reads at rest, so its scale error does not
  lower the trust.

## Variants & Generalizations

- A smooth trust curve, such as a Gaussian of the relative error, instead of the linear ramp.
- The full roll and pitch from one vector, as in tilt-compensated compasses.
- Trust from the gyroscope rate as well, to catch sideways accelerations.

## Applications

- Self-balancing robots and inverted pendulums.
- Attitude references for drones and handheld devices.

## Connections to Other Algorithms

- [Complementary Filter](ComplementaryFilter.md): blends this angle with the integrated gyroscope rate.
- [Madgwick/Mahony AHRS](AhrsMadgwickMahony.md): full three-axis attitude from the same measurements.

## References & Further Reading

- M. Pedley, "Tilt Sensing Using a Three-Axis Accelerometer", Freescale AN3461, 2013.
- R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group", IEEE
  Transactions on Automatic Control 53(5), 2008.
