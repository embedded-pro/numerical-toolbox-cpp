# IIR Filter Design (Butterworth / Chebyshev-I + Bilinear Transform)

## Overview & Motivation

Embedded systems often require adaptive filtering — adjustable cutoff frequencies, runtime-selected
filter orders, or field-reconfigurable equalizer bands — without a host toolchain to bake in
coefficients. This algorithm computes second-order-section (SOS) biquad coefficients for
Butterworth and Chebyshev Type-I IIR filters entirely on the device, using only fixed-point-safe
arithmetic and bounded arrays.

The design pipeline converts a normalized analog prototype (unit-cutoff Butterworth or Chebyshev
ellipse of poles) to a digital filter at any requested cutoff, applying the bilinear transform with
pre-warping to guarantee that the digital cut-off frequency lands exactly on the requested value.

## Mathematical Theory

### Analog Prototype Poles

#### Butterworth

The $N$th-order Butterworth prototype places poles uniformly on the left half of the unit circle in
the $s$-plane:

$$s_k = e^{j\pi(2k + N + 1)/(2N)}, \quad k = 0, 1, \ldots, N-1$$

All poles have magnitude 1 and lie strictly in the open left half-plane, guaranteeing stability and
a maximally flat (Butterworth) magnitude response.

#### Chebyshev Type-I

Chebyshev-I poles lie on an ellipse in the $s$-plane. Given ripple $R_p$ dB:

$$\varepsilon = \sqrt{10^{R_p/10} - 1}$$

$$\alpha = \frac{1}{N}\sinh^{-1}\!\left(\frac{1}{\varepsilon}\right)$$

$$s_k = -\sinh(\alpha)\sin\theta_k + j\cosh(\alpha)\cos\theta_k, \quad \theta_k = \frac{\pi(2k+1)}{2N}$$

This trades the maximally-flat passband of Butterworth for equiripple passband with steeper
transition-band roll-off at the same order.

### Frequency Pre-warping

The bilinear transform compresses the entire analog frequency axis $(-\infty, +\infty)$ onto the
digital unit circle non-linearly. To ensure the digital cutoff $f_c$ maps exactly to the correct
position after this compression, the analog prototype cutoff $\omega_c$ is pre-warped:

$$\omega_c = 2 f_s \tan\!\left(\frac{\pi f_c}{f_s}\right)$$

### Analog Frequency Transforms

A normalized (LP cutoff = 1 rad/s) prototype is frequency-transformed before bilinear mapping:

- **Low-pass:** $s \leftarrow s / \omega_c$ — scales the prototype to the target cutoff.
- **High-pass:** $s \leftarrow \omega_c / s$ — maps the unit-circle pole locations to HP positions.
- **Band-pass:** $s \leftarrow (s^2 + \omega_0^2) / (B \cdot s)$ — doubles the order, centers
  the band at $\omega_0$ with bandwidth $B$.
- **Band-stop:** the inverse band-pass mapping.

### Bilinear Transform

Each analog root $s_k$ is mapped to the $z$-plane:

$$z_k = \frac{1 + s_k / (2 f_s)}{1 - s_k / (2 f_s)}$$

This is an exact Möbius transformation that maps the left $s$-half-plane to the interior of the
unit circle, preserving stability.

### Second-Order Sections

Complex-conjugate pole pairs $(z_k, z_k^*)$ are combined into real-coefficient biquad sections:

$$H_i(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$

with $a_1 = -2\,\mathrm{Re}(z_k)$ and $a_2 = |z_k|^2$. A single real pole yields a first-order
section padded as a biquad with $b_2 = a_2 = 0$.

## Complexity Analysis

| Case   | Time     | Space          | Notes                       |
|--------|----------|----------------|-----------------------------|
| Design | $O(N)$   | $O(N)$ static  | One-off; no per-sample cost |
| Filter | $O(N/2)$ | $O(N/2)$ state | Delegated to BiquadCascade  |

All arrays are bounded by the compile-time `MaxOrder` template parameter; no heap is used.

## Step-by-Step Walkthrough

**Example:** 2nd-order Butterworth LP, $f_c = 100$ Hz, $f_s = 1000$ Hz.

1. **Prototype poles** ($N=2$): $s_{0,1} = e^{j\cdot 3\pi/4}, e^{j\cdot 5\pi/4} = \frac{-1 \pm j}{\sqrt{2}}$

2. **Pre-warp:** $\omega_c = 2 \times 1000 \times \tan(\pi \times 100 / 1000) \approx 726.5$ rad/s

3. **Scale LP:** $s_{0,1} \leftarrow s_{0,1} \times \omega_c$

4. **Bilinear map:** each scaled pole $\to z_{0,1}$ inside the unit circle.

5. **SOS:** conjugate pair forms one biquad with real coefficients $b_0, b_1, b_2, a_1, a_2$.

6. **Gain normalization:** DC gain adjusted so $|H(0)| = 1$.

## Pitfalls & Edge Cases

- **Cutoff near Nyquist:** $\tan(\pi f_c / f_s) \to \infty$ as $f_c \to f_s/2$; the design should
  reject $f_c \geq f_s/2$.
- **Very low cutoff:** pre-warp is stable, but pole magnitudes approach 1, risking coefficient
  quantization artifacts in fixed-point implementations.
- **High Chebyshev ripple or order:** large $\varepsilon$ causes the ellipse to become very flat;
  pole angles cluster, potentially causing paired sections to be nearly identical and numerically
  sensitive.
- **Band-pass/stop order doubling:** a 4th-order BP prototype yields 8 digital poles — verify that
  `MaxOrder` accounts for the doubled count.
- **Odd order:** one real analog pole transforms to a first-order digital section; it must be padded
  to a biquad with zero second-order terms.

## Variants & Generalizations

- **Chebyshev Type-II / Elliptic:** add zeros on the imaginary axis (Type-II) or place both poles
  and zeros on the ellipse (elliptic/Cauer) for even steeper roll-off at the cost of stopband
  ripple.
- **Bessel/Thomson:** pole placement optimizes group-delay flatness rather than magnitude; useful for
  phase-sensitive systems.
- **Higher precision:** `double` instantiation (via the generic `T`) improves coefficient accuracy
  for very high order or narrowband designs before quantizing to `float`.

## Applications

- Adaptive sensor bandwidth control (temperature, pressure, inertial sensors).
- Runtime-reconfigurable equalizer banks in audio processing.
- Anti-aliasing pre-filters with software-selectable cutoff.
- Self-calibrating instrumentation where the signal bandwidth is measured at startup.

## Connections to Other Algorithms

- **BiquadCascade** — the runtime consumer of the emitted SOS coefficients.
- **NotchCombFilter** — a band-stop specialization directly parameterized by frequency and Q;
  simpler but less flexible than a full band-stop IIR design.
- **FIR** — linear phase, always stable, but requires much higher order for the same roll-off
  slope; IIR is preferred when group-delay distortion is acceptable.

## References & Further Reading

- T. W. Parks, C. S. Burrus, *Digital Filter Design*, Wiley, 1987.
- A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Pearson, 2010.
- R. G. Lyons, *Understanding Digital Signal Processing*, 3rd ed., Prentice Hall, 2011.
