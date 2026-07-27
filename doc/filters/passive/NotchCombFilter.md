# Notch / Comb Filter

## Overview & Motivation

Narrow-band interference — mains hum at 50/60 Hz and its harmonics — contaminates bio-signal
acquisition (ECG, EEG, EMG) and precision instrumentation. A notch filter surgically eliminates
a single frequency while leaving the rest of the spectrum intact. A comb filter extends this idea
to an entire harmonic series using a single delay line, making it far more efficient than cascading
individual notches when the interference is periodic.

## Mathematical Theory

### Notch Filter (RBJ Biquad)

The continuous-time transfer function for a band-reject filter with centre frequency $f_0$ and
quality factor $Q$ is:

$$H(s) = \frac{s^2 + \omega_0^2}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2}$$

Applying the bilinear transform with $\omega_0 = 2\pi f_0 / f_s$, the discrete-time biquad
coefficients (Bristow-Johnson "Cookbook" notch) are:

$$\alpha = \frac{\sin(\omega_0)}{2Q}, \quad c_\omega = \cos(\omega_0)$$

$$b_0 = b_2 = \frac{1}{1+\alpha}, \quad b_1 = \frac{-2c_\omega}{1+\alpha}$$

$$a_1 = \frac{-2c_\omega}{1+\alpha}, \quad a_2 = \frac{1-\alpha}{1+\alpha}$$

The zeros lie exactly on the unit circle at $e^{\pm j\omega_0}$, guaranteeing infinite attenuation
at $f_0$ in exact arithmetic.

Transposed Direct Form II (TDF-II) realises this with two state variables $z_1, z_2$:

$$y[n] = b_0 x[n] + z_1[n-1]$$
$$z_1[n] = b_1 x[n] - a_1 y[n] + z_2[n-1]$$
$$z_2[n] = b_2 x[n] - a_2 y[n]$$

### Comb Filter

A comb filter exploits the identity: subtracting (or adding) a signal delayed by $D$ samples
creates periodic nulls (or peaks) spaced every $f_s / D$ Hz:

**Feedforward (FIR) comb:**

$$y[n] = x[n] - g \cdot x[n-D]$$

Zeros at $\omega = 2\pi k / D$ for $k = 0, 1, \ldots, D-1$. The system is always stable.

**Feedback (IIR) comb:**

$$y[n] = x[n] + g \cdot y[n-D]$$

Poles at the same locations. The system is stable if and only if $|g| < 1$.

## Complexity Analysis

| Case  | Time   | Space  | Notes                             |
|-------|--------|--------|-----------------------------------|
| Notch | $O(1)$ | $O(1)$ | Fixed biquad; 2 multiplies/adds   |
| Comb  | $O(1)$ | $O(D)$ | Single circular-buffer read/write |

## Step-by-Step Walkthrough

**Notch example** — $f_0 = 50$ Hz, $f_s = 1000$ Hz, $Q = 10$:

1. $\omega_0 = 2\pi \cdot 0.05 \approx 0.3142$
2. $c_\omega = \cos(0.3142) \approx 0.9511$, $\alpha = \sin(0.3142)/(20) \approx 0.01564$
3. $a_0 = 1.01564$; divide all coefficients by $a_0$
4. $b_0 = b_2 \approx 0.9846$, $b_1 = a_1 \approx -1.8730$, $a_2 \approx 0.9693$
5. At each sample: compute $y = b_0 x + z_1$, then update $z_1, z_2$.

**Comb example** — $D = 4$, $g = 1$ (FIR):

Impulse response: $[1, 0, 0, 0, -1, 0, 0, 0, 0, \ldots]$ — zeros at 0, fs/4, fs/2, 3fs/4.

## Pitfalls & Edge Cases

- **High Q instability**: a very narrow notch ($Q \gg 10$) places poles close to the unit circle.
  Finite-precision arithmetic can shift them outside, causing instability. Use double precision or
  second-order-section cascades when $Q > 30$.
- **Detuned comb**: if $f_s$ is not an exact integer multiple of the harmonic spacing, the comb
  nulls misalign and leave residual hum. Prefer a sample rate that divides evenly, or cascade
  individual biquad notches.
- **Feedback comb stability**: gain must satisfy $|g| < 1$ with margin. Values near 1 produce
  extremely long ring-down and amplify quantisation noise.
- **DC gain**: the RBJ notch has unity gain at DC and Nyquist; FIR comb has gain $(1 - g)$ at DC.

## Variants & Generalizations

- **Notch with adjustable bandwidth**: replace the fixed $Q$ with an adaptive estimator tracking
  the instantaneous interference frequency (e.g., a phase-locked loop).
- **All-pass + notch cascade**: derive a steep-skirt band-stop by cascading a notch with an
  all-pass tuned to the same frequency.
- **Comb as reverberator**: adding (instead of subtracting) a delayed, attenuated copy produces
  artificial reverberation (Schroeder comb).
- **Multi-rate comb**: operate the comb at a lower decimated rate, then interpolate — reduces
  delay-line length proportionally.

## Applications

- Bio-signal acquisition: removing 50/60 Hz mains hum and harmonics from ECG/EEG/EMG.
- Power-line harmonic suppression in current/voltage sensors.
- Audio effects: chorus, flange, reverberation.
- Synchronous sampling: align comb nulls with switching-converter ripple harmonics.

## Connections to Other Algorithms

- **BiquadCascade** — the notch is a single biquad section; a cascade realizes higher-order
  band-stop designs with better numerical conditioning.
- **IirFilterDesign** — a band-stop Butterworth/Chebyshev design from a bilinear transform
  produces a family of notches; the RBJ notch here is the minimal single-section special case.
- **GoertzelAlgorithm** — detects the frequency that the notch removes; useful for adaptive
  centre-frequency tuning.

## References & Further Reading

- R. Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients",
  <https://www.w3.org/TR/audio-eq-cookbook/>
- R. G. Lyons, *Understanding Digital Signal Processing*, 3rd ed., Prentice Hall, 2011, Ch. 6.
- A. V. Oppenheim & R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Ch. 5.
