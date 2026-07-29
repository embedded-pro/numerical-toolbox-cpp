# Roadmap — Pseudocode Specifications

Design-level pseudocode for every proposed component in [../ROADMAP.md](../ROADMAP.md).
These are **specifications, not compilable code** — they describe *what* to build and *how*
the algorithm works, so an implementer can produce the real templated C++ afterwards.

## Convention

The tree mirrors the `numerical/` layout. Each algorithm gets its own folder with **three files**:

```
roadmap/<domain>/<AlgorithmName>/
├── implementation.md   # data structures, interface, algorithm pseudocode, complexity, float notes, deployment
├── tests.md            # GoogleTest test plan in pseudocode (TEST_F on float, StrictMock, no heap)
└── explanation.md      # short plain-language overview + the reference paper
```

All pseudocode respects the library constraints: **no heap**, bounded containers / `std::array`,
`OPTIMIZE_FOR_SPEED` on hot paths, and — per the current **float-only** decision — a generic
`template<typename T>` interface that is validated and instantiated for **`float`** only
(no `Q15`/`Q31`). The generic signature keeps a future fixed-point specialisation cheap to add.

The canonical worked example is
[filters/passive/ExponentialMovingAverage](filters/passive/ExponentialMovingAverage/implementation.md).

## Deployment shape (float-only)

Each spec maps to this concrete artifact set when deployed into `numerical/`:

| Spec file           | Deploys to                                                              |
|---------------------|-------------------------------------------------------------------------|
| `implementation.md` | `numerical/<domain>/<Name>.hpp` + `<Name>.cpp` (coverage instantiation) |
| `tests.md`          | `numerical/<domain>/test/Test<Name>.cpp`                                |
| `explanation.md`    | `doc/<domain>/<Name>.md` (expanded to follow `doc/TEMPLATE.md`)         |

Header shape (mirroring existing components such as `Fir.hpp`):

```cpp
#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"

namespace <ns>
{
    template<typename T /*, std::size_t sizes... */>
    class <Name>
    {
        static_assert(std::is_floating_point_v<T>, "<Name> supports floating-point types");
    public:
        // OPTIMIZE_FOR_SPEED on hot paths (Filter/Compute/Update/Solve/Step)
    };

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class <Name><float /*, sizes... */>;
#endif
}
```

Coverage `.cpp`:

```cpp
#include "numerical/<domain>/<Name>.hpp"
namespace <ns> { template class <Name><float /*, sizes... */>; }
```

CMake wiring:
- add `<Name>.hpp` to `target_sources(numerical.<target> PRIVATE ...)`
- add `<Name>.cpp` to `numerical_add_coverage_sources(numerical.<target> ...)`
- add `Test<Name>.cpp` to the `_test` target's `target_sources`

Tests: single-type **`TEST_F` on `float`**, `StrictMock` only, no heap; validate reference vectors
with `EXPECT_NEAR` and `math::Tolerance<float>()` (or an explicit tolerance).

> **Float-only note.** This diverges from the repo's historical "all three numeric types" mandate
> in `copilot-instructions.md` / `CLAUDE.md`. The `template<typename T>` signature is retained so
> `Q15`/`Q31` can be re-enabled per algorithm later by relaxing the `static_assert` and adding
> instantiations.

## Index

### `filters/passive`
`ExponentialMovingAverage` (1) · `MovingAverage` (2) · `MedianFilter` (6) · `CicFilter` (14) · `BiquadCascade` (15) · `NotchCombFilter` (16) · `SavitzkyGolayFilter` (22) · `IirFilterDesign` (45)

### `filters/active`
`AlphaBetaFilter` (8) · `ComplementaryFilter` (9) · `AhrsMadgwickMahony` (33) · `SquareRootKalmanFilter` (39)

### `controllers`
`SaturationRateLimiter` (3) · `BangBangHysteresis` (4) · `Feedforward2Dof` (7) · `GainScheduledController` (10) · `LeadLagCompensator` (17) · `LuenbergerObserver` (19) · `IntegralStateFeedbackLqi` (20)

### `robust_control`
`SlidingModeControl` (34) · `DisturbanceObserver` (35) · `ActiveDisturbanceRejection` (36) · `HInfinityStateFeedback` (46)

### `nonlinear_control`
`FeedbackLinearization` (40) · `BacksteppingControl` (41) · `ModelReferenceAdaptiveControl` (47)

### `analysis`
`SignalDetectors` (5) · `ConvolutionCorrelation` (11) · `GoertzelAlgorithm` (13) · `RealFastFourierTransform` (25) · `HilbertTransform` (37) · `DiscreteWaveletTransform` (38)

### `estimators/offline`
`PolynomialFitting` (12) · `TotalLeastSquares` (44) · `DynamicParameterIdentification` (M22)

### `estimators/online`
`LmsAdaptiveFilter` (21) · `MomentumObserver` (M16)

### `math`
`Quaternion` (18) · `Cordic` (23) · `MatrixExponential` (29) · `ContinuousToDiscrete` (30) · `SE3Transform` (M6)

### `control_analysis`
`ControllabilityObservability` (26) · `TransferFunctionStateSpace` (32)

### `solvers`
`RungeKuttaIntegrators` (24) · `QrDecomposition` (27) · `LuDecomposition` (28) · `LyapunovSylvester` (31) · `JacobiEigenSolver` (42) · `SingularValueDecomposition` (43)

### `trajectory`
`PolynomialTrajectory` (M2) · `TrapezoidalProfile` (M3) · `SCurveProfile` (M9) · `CartesianSlerpInterpolation` (M10) · `TimeOptimalPathParameterization` (M27)

### `controllers/manipulator`
`PdGravityCompensation` (M5) · `ComputedTorqueControl` (M12) · `ImpedanceControl` (M17) · `OperationalSpaceControl` (M18) · `HybridPositionForceControl` (M19) · `SlotineLiAdaptiveControl` (M20) · `CableTensionDistribution` (M25)

### `kinematics`
`DenavitHartenberg` (M7) · `SpatialJacobian` (M8) · `ManipulabilityIndex` (M11) · `PoseInverseKinematics` (M13) · `RedundancyResolution` (M14) · `ProductOfExponentials` (M15) · `AnalyticalIkPieper` (M21) · `ParallelManipulatorKinematics` (M23) · `MobileManipulatorKinematics` (M24) · `ContinuumKinematics` (M26)

### `dynamics`
`GenericJointLink` (M1) · `FrictionCompensation` (M4)
