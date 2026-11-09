# Fast Trigonometry

## Overview & Motivation

A field-oriented motor controller evaluates a sine and a cosine of the rotor angle on every current-loop period, tens of thousands of times a second, on processors whose floating-point unit has no transcendental instructions. A library sine costs hundreds of cycles; a table lookup with linear interpolation costs a few dozen and is accurate to well under one percent, which is below the resolution of the current measurement it is used with. Calibration and identification code, which runs outside the control loop but must agree with it to float precision, needs a second, more accurate evaluation that still reuses the same table.

## Mathematical Theory

### Table

The table holds $N = 512$ samples of one period of the sine,

$$s_k = \sin\left(\frac{2\pi k}{N}\right), \qquad k = 0, \dots, N-1,$$

so the step between entries is $h = 2\pi / N$. Because $N$ is a power of two, an index is reduced modulo $N$ with a bit mask, which makes every angle — negative or many turns large — a valid input without a separate wrap.

### Interpolated sine

For an angle $\theta$, let $x = \theta / h$, $k = \lfloor x \rfloor$ and $f = x - k \in [0, 1)$. The fast sine interpolates linearly between neighbouring entries:

$$\widetilde{\sin}\,\theta = s_{k \bmod N} + f\,(s_{(k+1) \bmod N} - s_{k \bmod N}).$$

The cosine is the sine a quarter turn later, $\widetilde{\cos}\,\theta = \widetilde{\sin}(\theta + \pi/2)$.

### Accurate pair

The accurate pair rotates the table entry by the remaining angle $\delta = f h \in [0, h)$ instead of interpolating:

$$\sin\theta = s_k \cos\delta + c_k \sin\delta, \qquad \cos\theta = c_k \cos\delta - s_k \sin\delta,$$

where $c_k = s_{(k + N/4) \bmod N}$ is the table cosine. Because $\delta < h \approx 0.0123$, short Taylor series are enough:

$$\sin\delta \approx \delta\left(1 - \frac{\delta^2}{6}\right), \qquad \cos\delta \approx 1 - \delta^2\left(\frac{1}{2} - \frac{\delta^2}{24}\right).$$

### Error

Linear interpolation of the sine over a step $h$ has a worst-case error of $h^2/8 \approx 1.9\times10^{-5}$; the table entries are rounded to seven decimals, which adds at most $5\times10^{-8}$, so the fast sine and cosine are within about $2\times10^{-5}$ of the true values. The truncated series for the accurate pair leave errors of order $\delta^5/120 \approx 2\times10^{-12}$ and $\delta^6/720$, far below single-precision rounding, so the accurate pair is limited by the rounding of the table entries and of the arithmetic, a few units in the last place.

## Complexity Analysis

| Operation        | Time   | Space        | Notes                                                    |
|------------------|--------|--------------|----------------------------------------------------------|
| Fast sine/cosine | $O(1)$ | 2 KiB table  | One multiply, one floor, two table reads, one multiply-add |
| Accurate pair    | $O(1)$ | same table   | Two table reads and two short polynomials                |

The table is shared by both and lives in read-only memory.

## Step-by-Step Walkthrough

**Fast sine of $\theta = \pi/6$.** $x = (\pi/6)/(2\pi/512) = 512/12 \approx 42.667$, so $k = 42$ and $f \approx 0.667$. The table gives $s_{42} \approx 0.49290$ and $s_{43} \approx 0.50354$. The interpolated value is $0.49290 + 0.667 \times 0.01064 \approx 0.50000$.

**Negative angle, $\theta = -\pi/2$.** $x = -128$, so $k = -128$, $f = 0$, and $k \bmod 512 = 384$ by masking the two's-complement index. $s_{384} = -1$, the sine of $-\pi/2$.

**Accurate pair of $\theta = 0.5$.** $x \approx 40.744$, $k = 40$, $\delta = 0.744 \times h \approx 0.00913$. With $s_{40} \approx 0.47140$ and $c_{40} = s_{168} \approx 0.88192$, the rotation gives $\sin 0.5 \approx 0.47943$ and $\cos 0.5 \approx 0.87758$, both correct to float precision.

## Pitfalls & Edge Cases

**Large angles lose resolution** — the fraction $f$ is computed in floating point, so an angle of many thousand radians keeps fewer fractional bits. Wrap the angle at its source rather than accumulating it.

**The floor is computed by truncation and correction** — converting to an integer truncates toward zero, so negative inputs subtract one when the truncated value is above the input. Inputs beyond the 32-bit integer range are not supported.

**Accuracy, not just monotonicity** — the interpolated sine is continuous but its derivative jumps at every table entry. Use the accurate pair where a quantity is differentiated or compared against a model to float precision.

**Non-finite input** — infinities and NaN produce an undefined index; inputs must be finite.

## Variants & Generalizations

A larger table reduces the interpolation error quadratically ($h^2/8$); 256 entries quadruple it. Quadratic interpolation or a quarter-wave table trade memory for arithmetic. Polynomial approximations without a table, and the CORDIC iteration (see [CORDIC](Cordic.md)), avoid the table memory at the cost of more cycles per call.

## Applications

- **Park and inverse Park transforms** in field-oriented control, evaluated every current-loop period.
- **Space-vector modulation** and voltage feedforward rotations.
- **Calibration and identification** — the accurate pair reproduces a test signal or a plant model to float precision without a library call.

## Connections to Other Algorithms

[CORDIC](Cordic.md) computes the same functions iteratively without a table. [Angle Utilities](AngleUtilities.md) wraps angles before they grow large enough to cost resolution here.

## References & Further Reading

- J.-M. Muller, *Elementary Functions: Algorithms and Implementation*, 3rd ed., Birkhäuser, 2016 (table-based methods and range reduction)
