# Goertzel Algorithm

## Overview & Motivation

When a system needs to detect or measure a single known frequency — a DTMF touch-tone, a pilot sync tone, mains-hum, or a narrowband alarm — computing a full FFT wastes every cycle spent on bins that will never be read. The Goertzel algorithm computes exactly one DFT bin using a second-order recurrence that processes samples one at a time, requiring no input buffer and no twiddle-factor table. On the smallest microcontrollers with a few hundred bytes of RAM, this distinction is decisive.

## Mathematical Theory

### Core Definitions

Let $x[n]$, $n = 0, \ldots, N-1$, be a block of $N$ real samples. The DFT coefficient at bin $k$ is:

$$X[k] = \sum_{n=0}^{N-1} x[n]\, e^{-j2\pi kn/N}$$

Define the resonator coefficient:

$$c = 2\cos\!\left(\frac{2\pi k}{N}\right)$$

### Derivation

The DFT sum can be reorganised as the output of a single-pole IIR filter evaluated at $n = N$. Introduce the auxiliary sequence:

$$s[n] = x[n] + c\,s[n-1] - s[n-2], \quad s[-1] = s[-2] = 0$$

After the $N$ input samples, one further step with $x[N] = 0$ gives $s[N] = c\,s[N-1] - s[N-2]$, and the DFT bin follows:

$$X[k] = s[N] - s[N-1]\,e^{-j2\pi k/N}$$

Separating real and imaginary parts:

$$\mathrm{Re}\{X[k]\} = s[N] - s[N-1]\cos\!\left(\tfrac{2\pi k}{N}\right)$$

$$\mathrm{Im}\{X[k]\} = s[N-1]\sin\!\left(\tfrac{2\pi k}{N}\right)$$

The magnitude can be obtained without the final trigonometric products using the identity:

$$|X[k]|^2 = s[N-1]^2 + s[N-2]^2 - c\,s[N-1]\,s[N-2]$$

so $|X[k]| = \sqrt{s[N-1]^2 + s[N-2]^2 - c\,s[N-1]\,s[N-2]}$, saving two multiplies on the hot path.

### Frequency-to-Bin Mapping

A physical frequency $f_t$ sampled at $f_s$ maps to bin:

$$k = \mathrm{round}\!\left(\frac{f_t}{f_s}\,N\right)$$

Frequency resolution is $\Delta f = f_s / N$.

## Complexity Analysis

| Case       | Time   | Space  | Notes                                   |
|------------|--------|--------|-----------------------------------------|
| Per sample | $O(1)$ | $O(1)$ | One multiply, two adds, no table lookup |
| Full block | $O(N)$ | $O(1)$ | Two state words; trig only at block end |

For $B$ bins of interest the total cost is $O(BN)$. An FFT computing all $N$ bins costs $O(N\log N)$, so the Goertzel approach is cheaper when $B \ll \log_2 N$ — typically $B \le 4$ for a 256-point transform.

## Step-by-Step Walkthrough

Consider $N = 4$, $k = 1$, $x = [1, 0, -1, 0]$.

$c = 2\cos(\pi/2) = 0$

| $n$ | $x[n]$ | $s[n] = x[n] + 0\cdot s[n-1] - s[n-2]$ |
|-----|--------|----------------------------------------|
| 0   | 1      | $1 + 0 - 0 = 1$                        |
| 1   | 0      | $0 + 0 - 0 = 0$  (note: $s[-1]=0$)     |
| 2   | -1     | $-1 + 0 - 1 = -2$                      |
| 3   | 0      | $0 + 0 - 0 = 0$                        |

Extra step: $s[4] = 0\cdot s[3] - s[2] = -(-2) = 2$.

$\cos(2\pi/4) = 0$, $\sin(2\pi/4) = 1$

$\mathrm{Re}\{X[1]\} = s[4] - s[3]\cdot 0 = 2$

$\mathrm{Im}\{X[1]\} = s[3]\cdot 1 = 0$

Direct DFT check: $X[1] = 1 - (-1) = 2$.  Matches.

## Pitfalls & Edge Cases

The recurrence pole sits exactly on the unit circle, making the filter marginally stable. Error accumulates with each block if `Reset` is not called between blocks. The algorithm is designed for finite, bounded blocks of exactly $N$ samples.

Choosing $N$ such that the target tone lands near an integer bin minimises spectral leakage. A mismatch of half a bin can reduce the apparent magnitude by several decibels. Window functions do not apply here because the Goertzel filter already selects one bin; applying a window changes the effective DFT kernel.

The DC bin ($k = 0$) and the Nyquist bin ($k = N/2$) are purely real; their imaginary output is zero by symmetry.

## Variants & Generalizations

The magnitude-squared form $s_1^2 + s_2^2 - c\,s_1 s_2$ avoids the square root when only power detection (threshold comparison) is needed, saving a transcendental call on platforms without hardware `sqrt`.

When several tones must be tracked simultaneously, one independent Goertzel instance per tone runs in $O(BN)$ total with $O(B)$ state — still far cheaper than an FFT for small $B$.

A sliding Goertzel variant updates one sample at a time by subtracting the oldest sample, giving a continuous output at the cost of maintaining an input ring buffer; this trades $O(1)$ memory for $O(N)$ memory and is outside the scope of the no-heap constraint here.

## Applications

- DTMF decoding (telephony, industrial keypads)
- Sync-tone and pilot-tone detection in radio links
- Mains-frequency monitoring (50/60 Hz hum)
- Single-bin power spectral estimation in embedded monitors
- Lock-in detection for narrow-band sensor readout

## Connections to Other Algorithms

The Goertzel algorithm computes exactly the same result as one bin of the DFT computed by the Cooley-Tukey FFT (`FastFourierTransform`), but limits the computation to that single bin. When all $N$ bins are required simultaneously, the FFT is always faster. `PowerDensitySpectrum` builds on the FFT for broadband spectral estimation. `SignalDetectors` provides simpler, non-frequency-selective detectors (peak hold, RMS) that cost even less per sample.

## References & Further Reading

- G. Goertzel, "An Algorithm for the Evaluation of Finite Trigonometric Series," *American Mathematical Monthly*, 65(1), 34–35, 1958.
- A. V. Oppenheim and R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Prentice Hall, 2009, §8.1.
- P. Mock, "Add DTMF Generation and Decoding to DSP-µP Designs," *EDN*, March 1985 (practical embedded application of Goertzel).
