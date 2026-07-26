# CIC (Cascaded Integrator-Comb) Filter

## Overview & Motivation

Decimation and interpolation in digital signal processing typically require a lowpass anti-aliasing filter before the rate change. Standard FIR filters require multiplications proportional to their order. The CIC filter achieves a highly efficient lowpass response using only additions and subtractions, making it ideal for resource-constrained embedded systems where multipliers are expensive or unavailable.

CIC filters are used in sigma-delta ADC interfaces, software-defined radio front-ends, and any application that must drastically reduce the sample rate of a high-frequency signal stream before further processing.

## Mathematical Theory

### Core Definitions

A CIC decimator of order $N$ with decimation ratio $R$ and differential delay $M$ is defined by its $z$-domain transfer function:

$$H(z) = \left( \frac{1 - z^{-RM}}{1 - z^{-1}} \right)^N$$

The numerator factor $1 - z^{-RM}$ is the $z$-transform of the comb (differencing) stage. The denominator $\frac{1}{1 - z^{-1}}$ is the accumulator (integrator) stage.

### Structure

The filter consists of two cascaded sections:

**Integrator section** (running at the high input rate $f_s$): $N$ stages of first-order IIR accumulators,

$$y_k[n] = y_k[n-1] + y_{k-1}[n], \quad k = 1, \ldots, N$$

**Comb section** (running at the low output rate $f_s / R$): $N$ stages of differencing with delay $M$,

$$y_k[m] = y_{k-1}[m] - y_{k-1}[m - M], \quad k = 1, \ldots, N$$

### DC Gain

The unnormalized DC gain of the filter is:

$$G = (R \cdot M)^N$$

All outputs are divided by $G$ to normalize the DC gain to unity for a constant input.

### Frequency Response

The magnitude response in the baseband is approximately:

$$|H(f)| = \left| \frac{\sin(\pi f M R / f_s)}{R \sin(\pi f / f_s)} \right|^N$$

This is a sinc-like response that suppresses high-frequency content before the rate change.

## Complexity Analysis

| Case    | Time per input sample | Space          | Notes                                              |
|---------|-----------------------|----------------|----------------------------------------------------|
| Best    | $O(N)$                | $O(N \cdot M)$ | $N$ integrator ops; comb only at decimation points |
| Average | $O(N)$                | $O(N \cdot M)$ | Same                                               |
| Worst   | $O(N)$                | $O(N \cdot M)$ | Comb adds $N$ differencing ops at rate $f_s/R$     |

The integrator section executes $N$ additions per input sample. The comb section executes $N$ subtractions once every $R$ input samples. There are no multiplications in the signal path.

## Step-by-Step Walkthrough

Consider $N=2$, $R=4$, $M=1$, input impulse $x[0]=1$, $x[n]=0$ for $n>0$.

**Integrator section at $n=0,1,2,3$:**

| $n$ | Input | Integrator 1 | Integrator 2 |
|-----|-------|--------------|--------------|
| 0   | 1     | 1            | 1            |
| 1   | 0     | 1            | 2            |
| 2   | 0     | 1            | 3            |
| 3   | 0     | 1            | 4            |

**Comb section at decimated sample $m=0$ (triggered at $n=3$):**

- Comb 1 input: 4; delay buffer held 0; output: $4 - 0 = 4$; buffer updated to 4
- Comb 2 input: 4; delay buffer held 0; output: $4 - 0 = 4$; buffer updated to 4
- Normalized output: $4 / 16 = 0.25$

**At $m=1$ (triggered at $n=7$):**

- Integrator 2 output: 8 (accumulated four more 1s from integrator 1)
- Comb 1: $8 - 4 = 4$; Comb 2: $4 - 4 = 0$; Normalized output: $0.0$

## Pitfalls & Edge Cases

**Integer overflow in fixed-point implementations**: in fixed-point arithmetic, the integrators accumulate without bound between comb operations. Registers must be wide enough to hold $(R \cdot M)^N$ times the maximum input value. This implementation uses floating-point, which avoids this issue.

**Initial transient**: the filter takes several R-length blocks to settle to steady-state behavior for a constant input. DC normalization is exact only after the delay pipeline is fully flushed.

**Passband droop**: the sinc-shaped response causes attenuation even near DC as the input frequency increases. Compensation filters or a larger $R$ reduce in-band droop.

**Aliasing from high-order terms**: if the input signal has energy above $f_s / (2R)$, aliased components will appear at the output. A simple prefilter can reduce this.

## Variants & Generalizations

**Interpolating CIC**: the comb section runs at the low rate and the integrators at the high rate, acting as an upsampler. The architecture mirrors the decimator with sections swapped.

**Pruned CIC**: removes multiplier-free adder stages that contribute negligibly to the response, reducing hardware at the cost of response shape.

**Compensation filter**: a short linear-phase FIR appended at the low rate corrects passband droop without reintroducing multipliers in the high-rate path.

**Variable-rate CIC**: by making $R$ a runtime parameter, one filter structure supports multiple decimation ratios, useful in SDR front-ends.

## Applications

- Sigma-delta ADC decimation: the high oversampling rate (e.g., 256x) is reduced to Nyquist rate by a CIC stage before a compensation FIR.
- Software-defined radio: narrowband channels are extracted from a wideband stream by decimating with a CIC before channelization.
- Audio sample-rate conversion: high-to-low rate conversion with anti-aliasing, followed by a polyphase FIR for droop correction.
- Sensor interfaces: smoothing and downsampling of high-rate MEMS sensor data with minimal compute budget.

## Connections to Other Algorithms

A CIC filter of order $N=1$, $M=1$ is equivalent to a boxcar (rectangular window) FIR of length $R$, identical to a Moving Average filter operating on non-overlapping blocks. The Moving Average filter in this library is the continuous-output counterpart.

Higher-order CICs approximate a Gaussian response as $N \to \infty$, connecting them to the Gaussian filter family in theory.

The CIC is a special case of the more general Hogenauer filter structure, which can be pruned to reduce word widths at each stage.

## References & Further Reading

- E. B. Hogenauer, "An economical class of digital filters for decimation and interpolation," IEEE Transactions on Acoustics, Speech, and Signal Processing, vol. 29, no. 2, pp. 155-162, April 1981.
- R. E. Crochiere and L. R. Rabiner, "Multirate Digital Signal Processing," Prentice-Hall, 1983. Chapter 3.
- F. J. Harris, "Multirate Signal Processing for Communication Systems," Prentice-Hall, 2004.
