[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=coverage)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=embedded-pro_embedded-dsp-control&metric=duplicated_lines_density)](https://sonarcloud.io/summary/new_code?id=embedded-pro_embedded-dsp-control)

# Numerical Algorithms Library

## Overview

This library provides a comprehensive collection of digital signal processing (DSP), control algorithms, filters, optimizers, and estimators designed for robust and efficient signal analysis, manipulation, and system control. Developed to address complex technical challenges, the library offers a flexible and extensible framework for engineers, researchers, and developers working in signal processing, control systems, and related domains.

## Getting Started

Refer to the documentation and example scripts to quickly integrate and utilize the library's signal processing and control algorithms in your projects.

## Documentation

| Category                                       | Description                                                |
|------------------------------------------------|------------------------------------------------------------|
| [Analysis](doc/analysis/README.md)             | FFT, Power Spectral Density, DCT, Root Locus               |
| [Controllers](doc/controllers/README.md)       | PID, LQR                                                   |
| [Estimators](doc/estimators/README.md)         | Linear Regression, Yule-Walker                             |
| [Filters](doc/filters/README.md)               | Kalman Filter                                              |
| [Solvers](doc/solvers/README.md)               | Gaussian Elimination, Levinson-Durbin, Durand-Kerner, DARE |
| [Windowing](doc/windowing/README.md)           | Rectangular, Hamming, Hann, Blackman                       |
| [Neural Network](doc/neural_network/README.md) | Layers, activations, losses, optimizers, regularization    |

Each category page lists its algorithms with a brief description and links to the detailed documentation.

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


## Contributing

Contributions, issues, and feature requests are welcome. Please check the contributing guidelines before submitting pull requests.

## License

[Specify your license here - e.g., MIT, Apache 2.0]

