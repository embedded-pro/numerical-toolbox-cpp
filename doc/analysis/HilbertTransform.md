# Hilbert Transform / Analytic Signal

## Overview & Motivation

Every real-valued signal contains both positive and negative frequency components that carry redundant information. The Hilbert transform discards the negative half and pairs the original signal with a version shifted by exactly −90° at every frequency, producing a **complex analytic signal** whose magnitude tracks instantaneous amplitude and whose angle tracks instantaneous phase.

This is the standard technique for AM/DSB demodulation, vibration envelope detection, bearing-fault analysis, single-sideband modulation, and instantaneous-frequency measurement — all tasks where the underlying amplitude or phase varies slowly relative to a carrier and must be tracked in real time.

## Mathematical Theory

### Hilbert Transform

For a real signal $x(t)$ the Hilbert transform is the convolution

$$\mathcal{H}\{x\}(t) = \frac{1}{\pi} \, \text{p.v.} \int_{-\infty}^{\infty} \frac{x(\tau)}{t - \tau} \, d\tau$$

which is equivalently a multiplication of each frequency component by $-j \operatorname{sgn}(f)$, i.e. a $-90°$ phase rotation for positive frequencies and $+90°$ for negative.

### Analytic Signal

The analytic signal is

$$z(t) = x(t) + j\,\mathcal{H}\{x\}(t)$$

Its one-sided spectrum satisfies $Z(f) = 0$ for $f < 0$, $Z(0) = X(0)$, and $Z(f) = 2X(f)$ for $f > 0$.

### Instantaneous Attributes

| Quantity | Formula |
|----------|---------|
| Amplitude (envelope) | $A(t) = |z(t)| = \sqrt{x^2 + \mathcal{H}^2\{x\}}$ |
| Phase | $\phi(t) = \arg z(t) = \operatorname{atan2}(\mathcal{H}\{x\}, x)$ |
| Frequency | $f_i(t) = \frac{1}{2\pi}\frac{d\phi}{dt}$ (phase unwrapped before differencing) |

### FFT Method (Marple)

Given the $N$-point DFT $X[k]$ of a real sequence, the analytic signal is recovered by:

$$H[k] = \begin{cases} X[0] & k = 0 \\ 2X[k] & 1 \le k < N/2 \\ X[N/2] & k = N/2 \\ 0 & N/2 < k < N \end{cases}$$

followed by the inverse DFT of $H$.

### FIR Approximation

A Type III/IV antisymmetric FIR filter with impulse response

$$h[k] = \frac{2}{\pi k} w[k], \quad k \text{ odd}; \quad h[k] = 0, \quad k \text{ even}$$

(with a Hamming window $w[k]$) approximates the ideal $-90°$ phase shift across its passband. The delayed original signal (center-tap copy) is paired with the filtered output to form the approximate analytic signal.

## Complexity Analysis

| Method | Time per call | Space | Notes |
|--------|--------------|-------|-------|
| FFT (block) | $O(N \log N)$ | $2N$ complex words | Exact, latency $N$ |
| FIR (streaming) | $O(P)$ per sample | $P$ words state | Approx., latency $(P-1)/2$ |
| Feature extraction | $O(1)$ | None | atan2, sqrt, subtract |

$P$ = number of FIR taps.

## Step-by-Step Walkthrough

**FFT method on $N = 8$, input $x = \cos(2\pi n / 8)$:**

1. Forward DFT yields $X[1] = 4$, $X[7] = 4$, all others zero.
2. Apply one-sided weighting: $H[1] = 8$, $H[7] = 0$.
3. Inverse DFT of $H$ produces the imaginary part $\sin(2\pi n / 8)$.
4. Analytic signal: $z[n] = \cos(2\pi n/8) + j\sin(2\pi n/8)$, amplitude $\equiv 1$.

## Pitfalls & Edge Cases

- **DC and Nyquist bins** — their imaginary parts must remain zero; the one-sided formula preserves this.
- **Phase unwrapping** — before computing instantaneous frequency, the phase difference must be mapped to $(-\pi, \pi]$ to suppress $2\pi$ jumps.
- **Block-edge artefacts** — the FFT method treats the block as periodic; trim the first and last few samples when asserting accuracy.
- **FIR group delay** — the FIR output is delayed by $(P-1)/2$ samples relative to the input; align before comparing to the FFT method.
- **FIR bandwidth** — the FIR Hilbert approximation degrades near DC and Nyquist; use only over the filter's flat passband.

## Variants & Generalizations

- **Block FFT method** — exact, no ripple, requires a complete block; suitable for offline or buffered processing.
- **FIR streaming method** — causal, constant memory, approximate; length and window choice trade accuracy against latency.
- **Quadrature-oscillator method** — for narrowband signals, a simple IIR resonator can approximate the 90° shift with even lower cost.

## Applications

- AM/DSB envelope demodulation and AGC.
- Vibration and bearing-fault analysis (envelope of resonance band).
- Single-sideband (SSB) radio modulation/demodulation.
- Instantaneous frequency tracking in FM receivers and Doppler radar.
- Phase and frequency estimation in PLLs.

## Connections to Other Algorithms

- Uses `FastFourierTransform` / `FastFourierTransformRadix2Impl` as the block back-end.
- `RealFastFourierTransform` is an alternative back-end for purely real inputs.
- `ConvolutionCorrelation` provides the FIR convolution primitive used by the streaming path.
- `SignalDetectors` (RMS envelope, peak hold) offers cheaper non-coherent envelope estimation.

## References & Further Reading

- S. L. Marple, "Computing the Discrete-Time Analytic Signal via FFT," *IEEE Transactions on Signal Processing*, 47(9), 2600–2603, 1999.
- A. V. Oppenheim and R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Prentice Hall, Ch. 12.
- S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 9 (available free online).
