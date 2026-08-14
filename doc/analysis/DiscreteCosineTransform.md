# Discrete Cosine Transform (DCT)

## Overview & Motivation

The Discrete Cosine Transform projects a real-valued signal onto a set of cosine basis functions at different frequencies. Unlike the DFT, the DCT produces **purely real coefficients** and exhibits superior **energy compaction** — most of the signal's energy concentrates in a few low-frequency coefficients. This property makes it the backbone of lossy compression standards (JPEG, MPEG, AAC).

This library implements **DCT-II** (the most common variant) by leveraging the FFT: the input is rearranged, transformed via a real-valued FFT, and post-processed with twiddle factors. This approach inherits the FFT's $O(N \log N)$ efficiency rather than requiring a dedicated $O(N^2)$ computation.

## Mathematical Theory

### DCT-II Definition

This implementation uses the **orthonormal** DCT-II. For a length-$N$ sequence $x[n]$:

$$X[0] = \frac{1}{\sqrt{N}} \sum_{n=0}^{N-1} x[n]$$

$$X[k] = \sqrt{\frac{2}{N}} \sum_{n=0}^{N-1} x[n] \cos\!\left(\frac{\pi}{N}\left(n + \tfrac{1}{2}\right) k\right), \quad k = 1, \ldots, N-1$$

This normalisation makes the transform matrix unitary, so Parseval's theorem holds exactly: $\sum_k X[k]^2 = \sum_n x[n]^2$. The inverse (orthonormal DCT-III) recovers $x[n]$:

$$x[n] = \frac{X[0]}{\sqrt{N}} + \sqrt{\frac{2}{N}} \sum_{k=1}^{N-1} X[k] \cos\!\left(\frac{\pi}{N}\left(n + \tfrac{1}{2}\right) k\right)$$

### FFT-Based Computation

The DCT can be computed via the DFT of a reordered sequence:

1. Form a new sequence $y[n]$ by interleaving even-indexed and reverse odd-indexed samples of $x$.
2. Compute the $N$-point FFT: $Y[k] = \text{FFT}\{y\}$.
3. Apply twiddle factors and orthonormal scale:
   - $k = 0$: $X[0] = \text{Re}(Y[0]) / \sqrt{N}$
   - $k \ge 1$: $X[k] = \sqrt{2/N} \cdot \text{Re}\!\left(W[k] \cdot Y[k]\right)$, where $W[k] = e^{-j\pi k / 2N}$.

### Why Cosine Basis?

Cosine functions are **symmetric** (even functions). The DCT implicitly mirrors the signal rather than making it periodic, avoiding the boundary discontinuities that cause spectral leakage in the DFT. This is why DCT coefficients decay faster and energy compaction is better.

## Complexity Analysis

| Case | Time          | Space  | Notes                                                            |
|------|---------------|--------|------------------------------------------------------------------|
| All  | $O(N \log N)$ | $O(N)$ | Dominated by the internal FFT; twiddle post-processing is $O(N)$ |

The reordering step and twiddle multiplication are both linear, so the overall cost is determined by the FFT.

## Step-by-Step Walkthrough

**Input:** $x = [1, 2, 3, 4]$, $N = 4$

**Step 1 — Reorder for FFT**

Even-indexed positions filled left-to-right, odd-indexed filled right-to-left:

$$y = [x[0],\; x[2],\; x[3],\; x[1]] = [1, 3, 4, 2]$$

**Step 2 — Compute FFT**

$$Y = \text{FFT}([1, 3, 4, 2]) = [10,\; -3-j,\; 0,\; -3+j]$$

**Step 3 — Apply twiddle factors and orthonormal scale** ($N=4$, $W[k] = e^{-j\pi k/8}$)

| $k$ | $W[k]$         | $W[k] \cdot Y[k]$         | $X[k]$                                    |
|-----|----------------|---------------------------|-------------------------------------------|
| 0   | —              | —                         | $10 / \sqrt{4} = 5$                       |
| 1   | $e^{-j\pi/8}$  | $\approx -3.154 + 0.224j$ | $\sqrt{0.5} \cdot (-3.154) \approx -2.23$ |
| 2   | $e^{-j\pi/4}$  | $0$                       | $0$                                       |
| 3   | $e^{-j3\pi/8}$ | $\approx -0.224 + 3.154j$ | $\sqrt{0.5} \cdot (-0.224) \approx -0.16$ |

**Output:** $X \approx [5,\; -2.23,\; 0,\; -0.16]$

Notice how most of the energy is in $X[0]$ (the DC component) — energy compaction in action. The orthonormal scale ensures $\sum_k X[k]^2 = 1^2+2^2+3^2+4^2 = 30$.

## Pitfalls & Edge Cases

- **Power-of-2 length required** — inherited from the underlying FFT constraint.
- **Normalization convention.** This library uses the **orthonormal** convention ($1/\sqrt{N}$ for $k=0$, $\sqrt{2/N}$ for $k \ge 1$), which satisfies Parseval's theorem and makes the transform matrix unitary. Other references may use the unnormalized form; scale accordingly when interfacing.
- **Fixed-point overflow.** The reordering and FFT steps must preserve range; apply the 0.9999 scaling factor used throughout this library.
- **Inverse accuracy.** Rounding errors accumulate in the forward-then-inverse round-trip, especially for Q15 types.
- **Real input only.** Complex inputs are not supported by the reordering trick.

## Variants & Generalizations

| Variant            | Description                                                       |
|--------------------|-------------------------------------------------------------------|
| **DCT-I**          | Symmetric extension; used in some filter bank designs             |
| **DCT-III (IDCT)** | Inverse of DCT-II; used for reconstruction in JPEG decoding       |
| **DCT-IV**         | Self-inverse; basis of the MDCT used in MP3 and AAC               |
| **MDCT**           | Modified DCT with 50 % overlap; foundation of modern audio codecs |
| **2-D DCT**        | Applied row-then-column for image compression (JPEG 8×8 blocks)   |

## Applications

- **Image compression** — JPEG applies DCT to 8×8 pixel blocks; quantizing high-frequency coefficients achieves compression.
- **Audio compression** — MDCT (a DCT variant) is the core transform in MP3, AAC, and Opus codecs.
- **Feature extraction** — Mel-frequency cepstral coefficients (MFCCs) used in speech recognition are derived from a DCT.
- **Data reduction** — Truncating small DCT coefficients provides a compact signal representation for embedded storage.
- **Numerical methods** — DCT is used in fast solvers for certain partial differential equations (Poisson's equation on regular grids).

## Connections to Other Algorithms

```mermaid
graph LR
    DCT["Discrete Cosine Transform"]
    FFT["Fast Fourier Transform"]
    WIN["Window Functions"]
    FFT --> DCT
    WIN -.-> DCT
```

| Algorithm                                         | Relationship                                                              |
|---------------------------------------------------|---------------------------------------------------------------------------|
| [Fast Fourier Transform](FastFourierTransform.md) | DCT is computed via FFT with input reordering and twiddle post-processing |
| [Window Functions](../windowing/window.md)        | Some DCT applications use windowing to control boundary effects           |

## References & Further Reading

- Ahmed, N., Natarajan, T. and Rao, K.R., "Discrete Cosine Transform", *IEEE Transactions on Computers*, C-23(1), 1974.
- Rao, K.R. and Yip, P., *Discrete Cosine Transform: Algorithms, Advantages, Applications*, Academic Press, 1990.
- Makhoul, J., "A fast cosine transform in one and two dimensions", *IEEE Transactions on ASSP*, 28(1), 1980.
