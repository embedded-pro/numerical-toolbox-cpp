[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=coverage)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)

# Embedded Numerical Algorithms Library

## Overview

This library provides a comprehensive collection of digital signal processing (DSP), control algorithms, filters, optimizers, and estimators designed for robust and efficient signal analysis, manipulation, and system control. Developed to address complex technical challenges, the library offers a flexible and extensible framework for engineers, researchers, and developers working in signal processing, control systems, and related domains.

## Key Features

- **Advanced Signal Processing Techniques**: Comprehensive algorithms for signal analysis, filtering, transformation, and reconstruction
- **Sophisticated Control Algorithms**: Diverse range of control strategies including linear, non-linear, adaptive, and predictive control methods
- **Integrated DSP and Control Design**: Seamless integration of signal processing and control system approaches
- **High Performance**: Optimized implementations focusing on computational efficiency and numerical precision
- **Cross-Platform Compatibility**: Designed to work seamlessly across different computing environments

## Use Cases

This library is ideal for applications in:
- Robotics and autonomous systems
- Industrial process control
- Telecommunications
- Audio and video processing
- Scientific instrumentation
- Adaptive signal filtering
- Control system design and simulation
- Research and academic projects

## Getting Started

Refer to the documentation and example scripts to quickly integrate and utilize the library's signal processing and control algorithms in your projects.

## Theory and How to use

1. Control Systems
    - [PID](doc/controllers/Pid.md)

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

5. [Neural Network](doc/neural_network/NeuralNetwork.md)


## Contributing

Contributions, issues, and feature requests are welcome. Please check the contributing guidelines before submitting pull requests.

## License

[Specify your license here - e.g., MIT, Apache 2.0]
