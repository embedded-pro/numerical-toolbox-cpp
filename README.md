[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=coverage)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)

# Numerical Algorithms Library

## Overview

This library provides a comprehensive collection of digital signal processing (DSP), control algorithms, filters, optimizers, and estimators designed for robust and efficient signal analysis, manipulation, and system control. Developed to address complex technical challenges, the library offers a flexible and extensible framework for engineers, researchers, and developers working in signal processing, control systems, and related domains.

## Getting Started

Refer to the documentation to quickly integrate and utilize the library's signal processing and control algorithms in your projects.

## Documentation

| Category                                                           | Description                                                          |
|--------------------------------------------------------------------|----------------------------------------------------------------------|
| [Analysis](doc/analysis/README.md)                                 | FFT, Real-Input FFT (RFFT), Power Spectral Density, DCT, Discrete Wavelet Transform (Haar/Daubechies), Window Functions, Signal Detectors, Convolution & Correlation, Goertzel Algorithm, Decibels, Hilbert Transform / Analytic Signal |
| [Control Analysis](doc/control_analysis/README.md)                 | Frequency Response, Root Locus, Controllability/Observability Matrices & Gramians, Continuous-to-Discrete, Transfer Function ↔ State Space |
| [Controllers](doc/controllers/README.md)                           | Bang-Bang/Hysteresis, PID, LQR, LQI (Integral/Servo State Feedback), MPC, Saturation, Rate Limiter, Slew-Limited Saturation, Feedforward/2-DOF, Gain-Scheduled Controller, Lead-Lag Compensator, Luenberger Observer |
| [Estimators](doc/estimators/README.md)                             | Linear Regression, Polynomial Fitting, Total Least Squares, Yule-Walker (offline), Recursive Least Squares, LMS / NLMS Adaptive Filter (online), Consistency Metrics / NEES / NIS |
| [Filters](doc/filters/README.md)                                   | Kalman, Extended Kalman, Unscented Kalman, Square-Root Kalman, Alpha-Beta/Alpha-Beta-Gamma, FIR, IIR, Exponential Moving Average, Moving Average, Complementary, Median Filter, CIC (Cascaded Integrator-Comb), Notch/Comb Filter, Savitzky-Golay Filter, Biquad/Second-Order-Section Cascade, IIR Filter Design (Butterworth/Chebyshev-I + Bilinear Transform), Madgwick/Mahony AHRS |
| [Neural Network](doc/neural_network/README.md)                     | Layers, activations, losses, model                                   |
| [Optimization](doc/optimization/README.md)                         | Gradient Descent                                                     |
| [Regularization](doc/regularization/README.md)                     | L1 (Lasso), L2 (Ridge)                                              |
| [Math](doc/math/README.md)                                         | CORDIC, Quaternion, MatrixNorms, Step Response Metrics, MatrixExponential                             |
| [Solvers](doc/solvers/README.md)                                   | Gaussian Elimination, Levinson-Durbin, Durand-Kerner, Cholesky, DARE, Runge-Kutta ODE Integrators (RK4 + Dormand-Prince), Spectral Radius & Discrete Stability Margin, QR Decomposition (Householder / Givens), LU Decomposition with Partial Pivoting, Singular Value Decomposition (Golub-Kahan) |
| [Nonlinear Control](doc/nonlinear_control/README.md)               | Feedback Linearization, Backstepping Control, Model Reference Adaptive Control (MRAC) |
| [Robust Control](doc/robust_control/README.md)                     | Active Disturbance Rejection Control (ADRC + ESO), Sliding Mode Control (SMC), Disturbance Observer (DOB), H∞ State-Feedback Control |
| [Performance Optimization](doc/performance-optimization/README.md) | Compiler optimizations, SIMD                                         |

Each category page lists its algorithms with a brief description and links to the detailed documentation.

### Booklet

The entire documentation set is also published as a single book — read it online as a
[GitHub Pages site](https://embedded-pro.github.io/embedded-dsp-control/) or download the latest
PDF from the [Releases page](../../releases/latest). Both are generated automatically from `doc/`
(cover, Summary/table of contents, one chapter per category, consolidated references, back cover).

Build it locally with [Pandoc](https://pandoc.org) + XeLaTeX installed:

```bash
python scripts/build-booklet.py --format all   # writes build/booklet/{NumericalToolbox.pdf,index.html}
```

## Simulator

The `simulator/` directory contains interactive Qt-based GUI applications for visualizing and experimenting with the library's algorithms. These are desktop tools intended for development and exploration, separate from the core embedded-targeted library.

### Building the Simulator

The simulator requires Qt6 and is disabled by default. Enable it with:

```bash
cmake --preset host  # host preset enables it automatically
# or manually:
cmake -DNUMERICAL_TOOLBOX_BUILD_SIMULATOR=ON ...
```

Prerequisites: `qt6-base-dev` and `libgl1-mesa-dev` (Ubuntu/Debian).

### How to Run the Simulator

The simulator includes a Qt-based GUI for real-time visualization. Since the development environment runs inside a Dev Container, an X server on the host machine is required to display the GUI.

#### Host Setup

##### Windows

1. Install VcXsrv (free) or X410 from the Microsoft Store.
2. Launch VcXsrv with the following settings:
   - Multiple windows
   - Start no client
   - Disable access control (checked)
3. Open the project in VS Code and reopen in the Dev Container.

##### Linux

1. Allow Docker containers to access your X server:

```bash
xhost +local:docker
```

2. Open the project in VS Code and reopen in the Dev Container.

> **Note:** The Dev Container sets `DISPLAY=host.docker.internal:0.0` to forward GUI windows over TCP. On Linux, if you prefer Unix socket forwarding, you can override `DISPLAY` to `:0` inside the container and add a bind mount for `/tmp/.X11-unix`.


## Math Function Overrides

All math functions used by the library (`Sin`, `Cos`, `Abs`, `Sqrt`, `Pow`, …) are centralised in
`numerical/math/Math.hpp` under the `math::` namespace.
Each function ships with a default `constexpr` implementation that forwards to the corresponding
`std::` counterpart, but every one of them can be replaced at compile time with a
platform-specific implementation — CORDIC, hardware intrinsics, look-up tables, etc.

### How to override a function

**1. Define the override macro in CMake** (suppresses the default definition):

```cmake
target_compile_definitions(your_target PRIVATE MATH_SIN_OVERRIDE MATH_COS_OVERRIDE)
```

**2. Provide your own template definition** in a header included before any call site:

```cpp
// platform/PlatformMath.hpp
#pragma once
namespace math
{
    template<typename T>
    constexpr T Sin(T x) { return cordic_sin(static_cast<float>(x)); }

    template<typename T>
    constexpr T Cos(T x) { return cordic_cos(static_cast<float>(x)); }
}
```

**3. Include your header** before `numerical/math/Math.hpp` — or use a CMake forced-include so it
applies to every translation unit automatically:

```cmake
target_compile_options(your_target PRIVATE -include platform/PlatformMath.hpp)
```

### Available override macros

| Macro                    | Function         |
|--------------------------|------------------|
| `MATH_ABS_OVERRIDE`      | `math::Abs`      |
| `MATH_SQRT_OVERRIDE`     | `math::Sqrt`     |
| `MATH_SIN_OVERRIDE`      | `math::Sin`      |
| `MATH_COS_OVERRIDE`      | `math::Cos`      |
| `MATH_TAN_OVERRIDE`      | `math::Tan`      |
| `MATH_ASIN_OVERRIDE`     | `math::Asin`     |
| `MATH_ACOS_OVERRIDE`     | `math::Acos`     |
| `MATH_ATAN_OVERRIDE`     | `math::Atan`     |
| `MATH_ATAN2_OVERRIDE`    | `math::Atan2`    |
| `MATH_EXP_OVERRIDE`      | `math::Exp`      |
| `MATH_LOG_OVERRIDE`      | `math::Log`      |
| `MATH_LOG10_OVERRIDE`    | `math::Log10`    |
| `MATH_LOG2_OVERRIDE`     | `math::Log2`     |
| `MATH_POW_OVERRIDE`      | `math::Pow`      |
| `MATH_SINH_OVERRIDE`     | `math::Sinh`     |
| `MATH_COSH_OVERRIDE`     | `math::Cosh`     |
| `MATH_TANH_OVERRIDE`     | `math::Tanh`     |
| `MATH_HYPOT_OVERRIDE`    | `math::Hypot`    |
| `MATH_COPYSIGN_OVERRIDE` | `math::Copysign` |
| `MATH_FMOD_OVERRIDE`     | `math::Fmod`     |
| `MATH_CEIL_OVERRIDE`     | `math::Ceil`     |
| `MATH_FLOOR_OVERRIDE`    | `math::Floor`    |
| `MATH_ROUND_OVERRIDE`    | `math::Round`    |
| `MATH_ERFC_OVERRIDE`     | `math::Erfc`     |

Overrides are fully compile-time: the default body is excluded from the translation unit, so the
replacement is used even for inlined calls. Non-overridden functions continue to use the `std::`
defaults unchanged.

## Roadmap

Planned algorithms and components are tracked in [ROADMAP.md](ROADMAP.md) — a prioritized backlog
of DSP filters, controllers, estimators, and solvers ordered by implementation difficulty.

## Testing

Every algorithm is validated against the **mathematical invariants of its family** — not golden
output. A low-pass filter must attenuate above cutoff, an ODE integrator must reproduce a known
analytic solution to its order, a Kalman filter's covariance must stay positive-definite. Tests are
`TEST_F` on `float`, no heap, one behaviour per test, asserted against independent reference values.

The per-family metric strategy — which properties to assert for each algorithm type (accuracy,
frequency/transient response, stability, boundaries, invariants, convergence, statistical
consistency, conditioning) — is documented in **[TESTING.md](TESTING.md)**. Mechanics and framework
rules live in [.github/instructions/testing.instructions.md](.github/instructions/testing.instructions.md).

## Contributing

Contributions, issues, and feature requests are welcome. Please check the contributing guidelines before submitting pull requests.

## License

[Specify your license here - e.g., MIT, Apache 2.0]

