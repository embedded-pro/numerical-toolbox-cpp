# Root Locus Analysis

## Process Overview

```mermaid
graph TD
    A[Input: Open-Loop Numerator N(s), Denominator D(s)] --> B[Find Open-Loop Poles and Zeros]
    B --> C[Generate Logarithmic Gain Sweep]
    C --> D[For Each Gain K]
    D --> E[Form Characteristic Polynomial: D(s) + K·N(s)]
    E --> F[Find Roots using Durand-Kerner]
    F --> G[Store Roots as Locus Branches]
    G --> H{More Gain Steps?}
    H -->|Yes| D
    H -->|No| I[Compute Closed-Loop Poles at Current Gain]
    I --> J[Return Result]
```

## Mathematical Background

### Root Locus Definition

The root locus shows how the closed-loop poles of a feedback system move in the complex plane as a scalar gain $K$ varies. Given an open-loop transfer function:

$$G(s) = K \cdot \frac{N(s)}{D(s)}$$

the closed-loop characteristic equation is:

$$D(s) + K \cdot N(s) = 0$$

The root locus plots the solutions of this equation for $K \in [K_{\min}, K_{\max}]$.

### Properties

- **Starting points** ($K \to 0$): Roots converge to the open-loop poles (roots of $D(s)$)
- **Ending points** ($K \to \infty$): Roots converge to the open-loop zeros (roots of $N(s)$) or diverge to infinity
- **Number of branches**: Equal to the order of $D(s)$
- **Symmetry**: Complex roots appear in conjugate pairs for real-coefficient polynomials

### Gain Sweep

The gain is swept logarithmically to provide even resolution across decades:

$$K_i = 10^{\log_{10}(K_{\min}) + \frac{i}{N-1} \cdot (\log_{10}(K_{\max}) - \log_{10}(K_{\min}))}$$

where $N$ is `MaxGainSteps` and $i \in [0, N-1]$.

## API

```cpp
#include "numerical/analysis/RootLocus.hpp"

analysis::RootLocus<float, 5, 100> rootLocus;

// Open-loop transfer function: K * (s + 1) / (s^3 + 3s^2 + 2s)
std::array<float, 2> numerator = { 1.0f, 1.0f };      // s + 1
std::array<float, 4> denominator = { 1.0f, 3.0f, 2.0f, 0.0f }; // s^3 + 3s^2 + 2s

auto result = rootLocus.Calculate(
    infra::MakeRange(numerator),
    infra::MakeRange(denominator),
    1.0f,    // currentGain
    0.001f,  // gainMin
    50.0f);  // gainMax

// result.openLoopPoles:  roots of denominator
// result.openLoopZeros:  roots of numerator
// result.closedLoopPoles: roots of D(s) + currentGain * N(s)
// result.loci[branch][step]: complex root at each gain step
// result.gains[step]: gain value at each step
```

### Template Parameters

| Parameter | Description |
|-----------|-------------|
| `T` | Floating-point type (`float` or `double`) |
| `MaxOrder` | Maximum polynomial order (number of branches) |
| `MaxGainSteps` | Number of gain steps in the sweep (must be > 1) |

### Calculate Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `numerator` | — | Open-loop numerator polynomial coefficients (descending power) |
| `denominator` | — | Open-loop denominator polynomial coefficients (descending power) |
| `currentGain` | — | Gain at which to compute closed-loop poles |
| `gainMin` | 0.001 | Minimum gain for the sweep (must be > 0) |
| `gainMax` | 50 | Maximum gain for the sweep (must be > gainMin) |

### Preconditions

- `gainMin > 0` (logarithmic sweep requires positive gains)
- `gainMax > gainMin`
- `denominator.size() >= numerator.size()` (proper or strictly proper transfer function)
- `MaxGainSteps > 1` (enforced at compile time)

### Result Structure

| Field | Type | Description |
|-------|------|-------------|
| `loci` | `array<LociBranch, MaxOrder>` | Root positions for each branch at each gain step |
| `activeBranches` | `size_t` | Number of active locus branches (= denominator order) |
| `gains` | `GainVector` | Gain values at each step |
| `openLoopPoles` | `RootVector` | Poles of the open-loop system |
| `openLoopZeros` | `RootVector` | Zeros of the open-loop system |
| `closedLoopPoles` | `RootVector` | Poles at `currentGain` |
| `currentGain` | `T` | The current gain value |

### Complexity

- **Time**: $O(N \cdot n^2 \cdot k)$ where $N$ = gain steps, $n$ = polynomial order, $k$ = Durand-Kerner iterations
- **Space**: $O(N \cdot n)$ for storing all locus branches

## Numerical Considerations

- **Logarithmic gain sweep**: Provides uniform resolution across gain decades; requires strictly positive gains
- **Branch tracking**: Roots at each gain step are sorted by Durand-Kerner's default ordering (real part ascending). For smooth branch visualization, the UI may need to apply additional nearest-neighbor matching between consecutive steps
- **Polynomial addition**: `PolyAdd` pads the shorter polynomial (numerator) with leading zeros to align with the longer one (denominator), requiring `denominator.size() >= numerator.size()`
- **Root finding accuracy**: Inherits convergence properties from the Durand-Kerner solver; see [DurandKerner.md](DurandKerner.md) for details
