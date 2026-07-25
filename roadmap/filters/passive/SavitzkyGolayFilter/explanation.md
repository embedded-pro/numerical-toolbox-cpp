# Savitzky-Golay Filter — Overview

## What it is
An FIR smoother whose weights come from fitting a **low-order polynomial** to each sliding window by
least squares and reading off the centre value — or a chosen derivative — of that fit.

## Why it matters (embedded)
In spectroscopy, ECG/PPG, and any peak-bearing signal, a moving average is unacceptable because it
flattens and widens the very peaks you care about. Savitzky-Golay smooths the noise while preserving
peak height, width, and area, and it can output a clean derivative for free — all as a single
fixed-coefficient convolution.

## How it works (intuition)
Instead of averaging (which assumes the signal is locally constant), SG assumes the signal is
locally a polynomial of degree `Order`. Fitting that polynomial and evaluating it at the window
centre is a linear operation, so the whole procedure collapses into one fixed set of convolution
weights — computed once, at compile time. Picking the derivative row of the fit yields smoothed
first/second derivatives.

## Key parameters
- **Window (frame length, odd)** — more points ⇒ more noise reduction and more delay.
- **Order (polynomial degree)** — higher ⇒ less peak distortion but weaker smoothing.
- **Deriv** — 0 for smoothing, 1/2 for smoothed derivatives.

## Reference
A. Savitzky, M. J. E. Golay, "Smoothing and Differentiation of Data by Simplified Least Squares
Procedures," *Analytical Chemistry*, 36(8), 1964.

## See also
`MovingAverage` (flattens peaks), `MedianFilter` (spike rejection),
`PolynomialFitting` (the underlying least-squares idea).
