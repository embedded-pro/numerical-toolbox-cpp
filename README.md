[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=coverage)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)

# Numerical Algorithms Library

## Overview

This library provides a comprehensive collection of digital signal processing (DSP), control algorithms, filters, optimizers, and estimators designed for robust and efficient signal analysis, manipulation, and system control. Developed to address complex technical challenges, the library offers a flexible and extensible framework for engineers, researchers, and developers working in signal processing, control systems, and related domains.

## Getting Started

Refer to the documentation and example scripts to quickly integrate and utilize the library's signal processing and control algorithms in your projects.

## Theory and How to use

1. Control Systems
    - [PID](doc/controllers/Pid.md)
    - [LQR](doc/controllers/Lqr.md)

2. Analysis
    - [Fast Fourier Transform](doc/analysis/FastFourierTransform.md)
    - [Power Density Spectrum](doc/analysis/PowerDensitySpectrum.md)
    - [Discrete Cosine Transform](doc/analysis/DiscreteCosineTransform.md)

3. Filter
    - [Windowing](doc/windowing/window.md)
    - [Kalman Filter](doc/filters/active/KalmanFilter.md)

4. Estimators and Solvers
    - [Linear Regression](doc/estimators/LinearRegression.md)
    - [Yule-Walker (for AR and MA models)](doc/estimators/YuleWalker.md)
    - [Levinson-Durbin (Ax + B = 0 matrix solver)](doc/solvers/LevinsonDurbin.md)
    - [Gaussian Elimination](doc/solvers/GaussianElimination.md)
    - [Discrete Algebraic Riccati Equation](doc/solvers/DiscreteAlgebraicRiccatiEquation.md)

5. [Neural Network](doc/neural_network/NeuralNetwork.md)

## Simulator

The `simulator/` directory contains interactive Qt-based GUI applications for visualizing and experimenting with the library's algorithms. These are desktop tools intended for development and exploration, separate from the core embedded-targeted library.

### FFT Simulator

A graphical application for computing and visualizing Fast Fourier Transforms. Features:

- Configurable FFT size (64–4096 points)
- Adjustable sample rate
- Multi-component signal generation (frequency + amplitude per component)
- Real-time magnitude spectrum visualization

The simulator is split into two layers:

- **application/** — Pure C++ logic (signal generation, FFT computation). No Qt dependency.
- **view/** — Qt Widgets GUI (chart rendering, configuration panel, main window).

### Building the Simulator

The simulator requires Qt6 and is disabled by default. Enable it with:

```bash
cmake --preset host  # host preset enables it automatically
# or manually:
cmake -DNUMERICAL_TOOLBOX_BUILD_SIMULATOR=ON ...
```

Prerequisites: `qt6-base-dev` and `libgl1-mesa-dev` (Ubuntu/Debian).


## Contributing

Contributions, issues, and feature requests are welcome. Please check the contributing guidelines before submitting pull requests.

## License

[Specify your license here - e.g., MIT, Apache 2.0]

