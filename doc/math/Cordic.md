# CORDIC

## Overview & Motivation

Trigonometric functions, magnitudes, and vector rotations are fundamental operations in motor
control, radar processing, and navigation. On microcontrollers without a floating-point unit or
hardware multiplier, library implementations of `sin`, `cos`, `atan2`, and `hypot` require
expensive software-emulated multiplications. This limits their use in hard real-time loops where
deterministic execution time is mandatory.

CORDIC (COordinate Rotation DIgital Computer) solves this by expressing any planar rotation as a
sum of progressively smaller elementary rotations, each of which requires only a bit-shift and an
addition. The result is a trig engine that runs on shift-add hardware with a fixed, data-independent
cycle count — exactly what a deterministic real-time loop demands.

## Mathematical Theory

### Elementary Rotations

A rotation by angle $\theta$ in two dimensions transforms a vector $(x, y)$ to

$$x' = x\cos\theta - y\sin\theta, \quad y' = y\cos\theta + x\sin\theta.$$

Factoring out $\cos\theta$ gives

$$x' = \cos\theta\,(x - y\tan\theta), \quad y' = \cos\theta\,(y + x\tan\theta).$$

When $\tan\theta_i = \pm 2^{-i}$, the multiplication by $\tan\theta_i$ becomes a right-shift by $i$
bits. The angles $\theta_i = \arctan(2^{-i})$ form the **CORDIC angle table**.

### Rotation Mode (sin/cos)

Starting from $(x_0, y_0, z_0) = (K, 0, \theta)$, each iteration steers the residual angle $z$
toward zero:

$$x_{i+1} = x_i - \sigma_i \, 2^{-i} y_i$$
$$y_{i+1} = y_i + \sigma_i \, 2^{-i} x_i$$
$$z_{i+1} = z_i - \sigma_i \, \theta_i$$

where $\sigma_i = \text{sign}(z_i)$. After $N$ iterations, $x_N \approx \cos\theta$ and
$y_N \approx \sin\theta$.

### Vectoring Mode (atan2/magnitude)

Starting from $(x_0, y_0, z_0) = (x, y, 0)$, each iteration steers $y$ toward zero:

$$\sigma_i = -\text{sign}(y_i)$$

After $N$ iterations, $z_N \approx \arctan(y/x)$ and $x_N \approx \|(x, y)\| / K$.

### CORDIC Gain

Each elementary rotation stretches the vector length by $\sqrt{1 + 2^{-2i}}$. The accumulated
gain over $N$ iterations is

$$A_N = \prod_{i=0}^{N-1} \sqrt{1 + 2^{-2i}}.$$

The constant $K = 1/A_N \approx 0.6073$ compensates for this growth. In rotation mode the initial
$x$ is pre-scaled by $K$; in vectoring mode the final $x$ is multiplied by $K$.

### Convergence Domain

The convergence domain is $|z| \leq \sum_{i=0}^{N-1} \arctan(2^{-i})$. For $N = 16$ this exceeds
$\pi/2$, so inputs outside $[-\pi/2, \pi/2]$ must be range-reduced by shifting the angle by $\pm\pi$
and inverting the output signs. Vectoring mode uses quadrant detection on the signs of $x$ and $y$
to handle the full $[-\pi, \pi]$ range.

## Complexity Analysis

| Metric      | Value                                                 |
|-------------|-------------------------------------------------------|
| Time        | $O(N)$ — exactly $N$ shift-add steps per call         |
| Space       | $O(N)$ — angle table in ROM; $O(1)$ working registers |
| Cycle count | Fixed, data-independent — no branch on input value    |

One additional bit of precision is gained per iteration. $N = 16$ yields approximately 16-bit
accuracy; $N = 20$ reaches the limits of single-precision float.

## Step-by-Step Walkthrough

Compute $\sin(\pi/6) = 0.5$ with $N = 4$ for brevity (gain $K_4 \approx 0.6352$).

| $i$ | $\theta_i$ | $\sigma_i$ | $x_i$  | $y_i$  | $z_i$   |
|-----|------------|------------|--------|--------|---------|
| —   | —          | —          | 0.6352 | 0.0000 | 0.5236  |
| 0   | 0.7854     | +1         | 0.6352 | 0.6352 | −0.2618 |
| 1   | 0.4636     | −1         | 0.7940 | 0.3176 | 0.2018  |
| 2   | 0.2450     | +1         | 0.7147 | 0.5122 | −0.0432 |
| 3   | 0.1244     | −1         | 0.8425 | 0.4248 | 0.0812  |

After iteration 3: $y_4 \approx 0.43$, improving toward 0.5 as $N$ grows.

## Pitfalls & Edge Cases

The input to rotation mode must lie within the convergence domain after range reduction. Angles
at exactly $\pm\pi/2$ sit at the edge of the domain and may accumulate an extra half-ulp error.

In vectoring mode, $(x, y) = (0, 0)$ is degenerate; by convention the angle is returned as zero
rather than causing a division-by-zero or NaN.

The shift $2^{-i}$ eventually underflows in floating-point for large $i$; iterations beyond
$\lfloor -\log_2(\epsilon) \rfloor$ contribute nothing and can be capped without loss of accuracy.

## Variants & Generalizations

**Hyperbolic CORDIC** replaces the elementary angle table with $\tanh^{-1}(2^{-i})$ and handles
`sinh`, `cosh`, `exp`, and `ln`.

**Linear CORDIC** uses shifts alone (no rotation) to implement multiply and divide.

**Double-rotation trick** repeats certain iterations to extend the convergence domain to $(-\pi, \pi]$
without a range-reduction step.

## Applications

- Field-oriented motor control: Park/Clarke transforms require `sin`/`cos` at carrier frequency.
- Radar and sonar: Cartesian-to-polar conversion of sample streams.
- Navigation: continuous `atan2` for heading on heading-constrained MCUs.
- Audio synthesis: wavetable-free sine generation on FPU-less targets.

## Connections to Other Algorithms

`TrigonometricFunctions` provides a table-lookup alternative with lower iteration count but higher
ROM usage for the same precision. `Quaternion` consumes CORDIC-generated `sin`/`cos` for axis-angle
conversions. On targets with an FPU the standard library usually outperforms CORDIC; the advantage
is exclusive to multiply-poor hardware.

## References & Further Reading

- J. E. Volder, "The CORDIC Trigonometric Computing Technique," *IRE Transactions on Electronic Computers*, EC-8(3), pp. 330–334, 1959.
- R. Andraka, "A survey of CORDIC algorithms for FPGA-based computers," *Proc. ACM/SIGDA FPGA*, 1998, pp. 191–200.
- J. S. Walther, "A unified algorithm for elementary functions," *AFIPS Spring Joint Computer Conference*, 1971.
