# Mel Filterbank and MFCC

## Overview & Motivation

Keyword-spotting and small speech models on microcontrollers need a compact, perceptually motivated
description of a short audio frame. The log-mel spectrum and its decorrelated form, the
Mel-Frequency Cepstral Coefficients (MFCC), compress a frame of $N$ samples into a few tens of
values that follow the coarse spectral envelope on a frequency axis that matches human pitch
perception. Since Davis and Mermelstein (1980) the pipeline has been the standard speech front end,
and every stage has a fixed size, so it fits a no-heap, real-time budget.

The pipeline is: window → real FFT → power spectrum → triangular mel filterbank → floored natural
log (log-mel) → orthonormal DCT-II (MFCC, first $C$ coefficients).

## Mathematical Theory

### Mel scales

Two mel scales are in common use.

**HTK** (O'Shaughnessy):

$$m = 2595 \log_{10}\!\left(1 + \frac{f}{700}\right), \qquad f = 700\left(10^{m/2595} - 1\right)$$

**Slaney** (Auditory Toolbox, librosa default): linear below 1 kHz and logarithmic above, with
$f_{sp} = 200/3$ Hz per mel, a break point $f_b = 1000$ Hz ($m_b = f_b / f_{sp} = 15$) and step
$\lambda = \ln(6.4)/27$:

$$m = \begin{cases} f / f_{sp} & f < f_b \\ m_b + \ln(f / f_b)/\lambda & f \ge f_b \end{cases}
\qquad
f = \begin{cases} f_{sp}\, m & m < m_b \\ f_b\, e^{\lambda (m - m_b)} & m \ge m_b \end{cases}$$

Both pairs are exact inverses, so a Hz → mel → Hz round trip reproduces the input up to rounding.

### Triangular filterbank

For $M$ bands over $[f_\text{min}, f_\text{max}]$, place $M+2$ points equally spaced in mel and map
them back to Hz: $f_0 = f_\text{min} < f_1 < \dots < f_{M+1} = f_\text{max}$. Band $m$ rises from
$f_m$ to its centre $f_{m+1}$ and falls to $f_{m+2}$. FFT bin $k$ sits at $\nu_k = k f_s / N$:

$$H_m(k) = \max\!\left(0,\; \min\!\left(\frac{\nu_k - f_m}{f_{m+1} - f_m},\; \frac{f_{m+2} - \nu_k}{f_{m+2} - f_{m+1}}\right)\right)$$

**Sparse structure.** If $\nu_k$ lies in segment $j$, $[f_j, f_{j+1}]$, only two triangles are
non-zero there. Band $j$ rises with $r_k = (\nu_k - f_j)/(f_{j+1} - f_j)$ and band $j-1$ falls with
$1 - r_k$. Storing one segment index and one weight per bin is therefore enough, so applying the
filterbank costs one pass over the $N/2+1$ bins and needs no $M \times (N/2+1)$ weight matrix.

**Partition of unity.** Without normalisation, for every bin between the first centre $f_1$ and the
last centre $f_M$, the two active weights add up to $r_k + (1 - r_k) = 1$. This holds per bin and
exactly, not just in the continuous limit.

**Slaney (area) normalisation.** Each triangle is scaled by $2/(f_{m+2} - f_m)$, so its continuous
area is 1 and wide high-frequency bands do not dominate. The scale and the normalisation are
independent choices. With the Slaney scale, Slaney normalisation and the same $f_s, N, M,
f_\text{min}, f_\text{max}$, the weights equal librosa's `filters.mel` defaults.

### Log-mel and cepstrum

$$S_m = \sum_k H_m(k)\,|X_k|^2, \qquad \tilde S_m = \ln \max(S_m, \varepsilon)$$

The floor $\varepsilon$ bounds silent or band-limited frames at $\ln \varepsilon$ instead of
$-\infty$. The cepstrum is the orthonormal DCT-II, truncated to the first $C \le M$ terms:

$$c_k = s_k \sum_{m=0}^{M-1} \tilde S_m \cos\!\left(\frac{\pi k (2m+1)}{2M}\right), \qquad s_0 = \sqrt{1/M},\; s_{k>0} = \sqrt{2/M}$$

This is `scipy.fft.dct(·, type=2, norm='ortho')`. The $C \times M$ cosine basis and the $N$ window
coefficients are computed once at construction.

## Complexity Analysis

| Stage | Time | Memory (words) | Notes |
|---|---|---|---|
| Construction | $O(N + M + C M)$ | — | Mel points, bin→segment map, window, DCT basis |
| Window + real FFT | $O(N \log N)$ | $O(N)$ | Dominant cost |
| Power spectrum | $O(N)$ | $N/2+1$ | |
| Filterbank | $O(N)$ | $2(N/2+1) + M$ | Two weights per bin plus a gain per band |
| Log | $O(M)$ | $M$ | |
| DCT-II | $O(C M)$ | $C M$ | Direct product with the precomputed basis |

With $N=512$, $M=40$, $C=13$, the tables take about 1.6 k words, where a dense filterbank matrix
would need 10 k.

## Step-by-Step Walkthrough

$f_s = 16$ kHz, $N = 256$ ($\Delta\nu = 62.5$ Hz), $M = 20$, $C = 13$, $[20, 8000]$ Hz,
Slaney scale and normalisation, periodic Hann window, $\varepsilon = 10^{-10}$.

1. $m(20) = 0.3$ and $m(8000) = 45.25$, so the 22 mel points are spaced $2.14$ mel apart. The first
   band spans $20$–$305$ Hz with its centre at $162$ Hz.
2. Bin 2 ($125$ Hz) lies in segment 0, giving $r = (125 - 20)/(162 - 20) = 0.74$ for band 0. Bin 3
   ($187.5$ Hz) lies in segment 1: band 1 rises with $r = 0.17$ and band 0 falls with $0.83$.
3. Band 0's gain is $2/(305 - 20) = 7.0\cdot10^{-3}$. Its librosa row sum is $0.015754$.
4. The test frame (tones at 125, 440 and 1800 Hz plus a 100–7900 Hz chirp) gives
   $\tilde S \approx [-1.23, 0.30, 2.16, -0.32, -7.90, \dots, -5.76]$ and
   $c \approx [-13.82, 1.49, 2.08, 7.49, \dots]$, which match the float64 librosa + SciPy
   reference to within $5\cdot10^{-3}$.
5. An all-zero frame gives $\tilde S_m = \ln 10^{-10} = -23.03$ for every band, so
   $c_0 = \sqrt{20}\cdot(-23.03)$ and $c_{k>0} = 0$.

## Pitfalls & Edge Cases

- **Narrow low bands.** When $\Delta\nu$ is wider than the lowest triangles, a band can catch zero
  or one bin, and its energy comes from a single bin or is 0, in which case it is floored. Raise $N$
  or $f_\text{min}$, or lower $M$.
- **Floor choice.** $\varepsilon$ should sit below the quietest meaningful band energy, but above
  the float32 noise of the FFT, about $10^{-7}$ relative to the frame's peak power. Otherwise
  quiet bands report rounding noise instead of the floor.
- **Fast-math.** The stages use plain sums and products. Reassociation changes results only at the
  rounding level, and the explicit floor (rather than relying on $\ln 0$) keeps the output finite.
- **Scale and normalisation pairing.** Toolkits differ (HTK: HTK scale without normalisation;
  librosa: Slaney with Slaney). Match both choices when you compare against a reference.

## Variants & Generalizations

- **Log-mel / FBANK features** stop before the DCT. They are exposed alongside the cepstrum and are
  the usual input for CNN spectrogram models.
- **dB scaling** ($10 \log_{10}$) differs from the natural log by a constant factor of
  $10/\ln 10$, which the first layer of a learned model absorbs.
- **Liftering and deltas** (temporal derivatives over frames) are post-processing steps on the
  cepstrum sequence.

## Applications

Keyword spotting (DS-CNN / "Hello Edge" class models), speaker verification, audio event detection
and voice activity detection. A typical MCU configuration is 25–40 ms frames, a 10–20 ms hop,
$M = 40$ and $C = 10$–$13$.

## Connections to Other Algorithms

The stage consumes the half spectrum of the real-input FFT and the analysis windows. The cepstral
stage is a truncated DCT-II. Power-spectral-density estimation shares the window → FFT → $|X|^2$
front.

## References & Further Reading

- S. Davis, P. Mermelstein, "Comparison of parametric representations for monosyllabic word
  recognition in continuously spoken sentences," *IEEE Trans. ASSP* 28(4), 1980.
- M. Slaney, "Auditory Toolbox, Version 2," Interval Research Tech. Report 1998-010, 1998.
- S. Young et al., *The HTK Book*, ch. 5.
- Y. Zhang et al., "Hello Edge: Keyword Spotting on Microcontrollers," arXiv:1711.07128, 2017.
- B. McFee et al., "librosa: Audio and Music Signal Analysis in Python," *SciPy*, 2015.
