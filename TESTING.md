# Testing Strategy — Algorithm Metrics

The library's test strategy: every algorithm is validated against the **mathematical invariants of
its family**, not golden output. This is the reference for the `unit-tester` agent (and humans) when
writing **unit tests** for `numerical/`. It answers one question per algorithm: *which mathematical
properties must a correct implementation satisfy, and how do we assert them?*

## Rationale

A numerical algorithm is not validated by "it compiles and doesn't crash." Every family has a small
set of **characteristic invariants** — properties that hold for any correct implementation
regardless of parameters (a low-pass filter must attenuate above cutoff; an ODE integrator must
reproduce a known analytic solution to its order; a Kalman filter's covariance must stay
positive-definite). A unit test earns its place by pinning one such invariant against a **known
ground truth**, not by re-running the implementation and trusting its own output.

This file groups the library by **metric family** so a test author picks the right invariants fast:
FFT needs spectral/energy metrics, a PID needs transient-response metrics, a solver needs
residual/convergence metrics. Without this, tests drift toward shallow "golden output" snapshots that
pass while the math is wrong.

Canonical rules still apply ([AGENTS.md](AGENTS.md), [testing.instructions.md](.github/instructions/testing.instructions.md)):
`TEST_F` on `float`, one behaviour per test, no redundant cases, **no heap in tests**, assert with
`EXPECT_NEAR` + `math::Tolerance<float>()` against reference values.

## How the agent uses this

1. Identify the target algorithm's **family** (section below).
2. From that family's row, take the **applicable metric types** and author **one `TEST_F` per
   distinct property** — not per parameter permutation.
3. Prefer **analytic ground truth** (closed-form response, known transform pair, hand-solved system)
   over self-consistency. Fall back to a cross-method check (e.g. FFT vs direct DFT) only when no
   closed form exists.
4. Always include the cross-cutting metrics (accuracy, boundary, determinism/reset) plus the
   family-specific ones. Keep signal/data generation on the stack (`std::array`, bounded buffers).

## Metric types (the vocabulary)

| #  | Metric type                   | What it asserts                                              | Typical assertion                                                       |
|----|-------------------------------|--------------------------------------------------------------|-------------------------------------------------------------------------|
| M1 | **Numerical accuracy**        | output matches a closed-form / reference value               | `EXPECT_NEAR(out, ref, tol)`; ULP error for math funcs                  |
| M2 | **Frequency response**        | magnitude / phase / group delay vs analytic `H(e^{jω})`      | error in dB at DC, cutoff, Nyquist; passband ripple; stopband floor     |
| M3 | **Time / transient response** | step & impulse behaviour                                     | rise time, settling time, % overshoot, steady-state error               |
| M4 | **Stability**                 | poles/eigenvalues inside unit circle; BIBO; Riccati/Lyapunov | pole radius < 1; bounded long run; residual of Lyapunov/Riccati eq      |
| M5 | **Convergence**               | iterative process reaches the answer                         | iterations-to-tolerance; monotonic objective/residual; contraction rate |
| M6 | **Boundary / edge**           | zero, saturation, extreme magnitude, min sizes               | clamp limits; zero-in→zero-out; no NaN/Inf at extremes                  |
| M7 | **Invariants & conservation** | energy/Parseval, norm, probability mass, orthogonality       | Parseval residual; ‖q‖=1; Σsoftmax=1; energy drift bound                |
| M8 | **Statistical consistency**   | estimator bias / error / covariance sanity                   | RMSE vs truth; NEES/NIS in χ² band; R²; unbiasedness                    |
| M9 | **Conditioning / robustness** | behaviour under ill-conditioning & quantization              | residual growth vs condition number; quantization-error bound           |

## Family → metric-type matrix

| Family                                          | M1 | M2 | M3 | M4 | M5 | M6 | M7 | M8 | M9 |
|-------------------------------------------------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| Signal transforms (analysis)                    | ●  | ●  |    |    |    | ●  | ●  |    | ●  |
| Passive filters                                 | ●  | ●  | ●  | ●  |    | ●  |    |    |    |
| Stochastic/adaptive filters & online estimators | ●  |    | ●  | ●  | ●  | ●  | ●  | ●  |    |
| Controllers                                     | ●  | ○  | ●  | ●  |    | ●  |    |    |    |
| Control analysis                                | ●  | ●  | ●  | ●  |    | ●  |    |    |    |
| Offline estimators / regression                 | ●  |    |    |    | ●  | ●  |    | ●  | ●  |
| Optimization                                    | ●  |    |    |    | ●  | ●  |    |    |    |
| Regularization                                  | ●  |    |    |    |    | ●  | ●  |    |    |
| Solvers (linear / ODE / roots)                  | ●  |    | ○  | ●  | ●  | ●  | ●  |    | ●  |
| Neural network                                  | ●  |    |    |    | ○  | ●  | ●  |    |    |
| Math foundation                                 | ●  |    |    |    |    | ●  | ●  |    | ●  |

● primary   ○ situational

---

## Per-family detail

### 1. Signal transforms — `analysis/`
`FastFourierTransformRadix2Impl`, `RealFastFourierTransform`, `DiscreteCosineTransform`, `GoertzelAlgorithm`,
`ConvolutionCorrelation`, `PowerDensitySpectrum`, `SignalDetectors`, `windowing/`.

- **M1 accuracy** — known transform pairs: δ[n] → flat spectrum; single sinusoid → single bin at its
  frequency with correct magnitude; DC → energy only in bin 0.
- **M7 Parseval / energy** — `Σ|x|² ≈ (1/N)·Σ|X|²`; assert residual near 0.
- **M1 linearity** — `F(a·x + b·y) = a·F(x) + b·F(y)`.
- **M1 round-trip** — `Inverse(Forward(x)) ≈ x`; assert reconstruction RMSE.
- **M7 symmetry** — real input ⇒ conjugate-symmetric spectrum (RealFastFourierTransform): `X[N-k] = conj(X[k])`.
- **Convolution** — matches the direct sum; `x * δ = x`; commutativity; output length.
- **PSD** — non-negative; total power = signal variance; spectral peak at the tone's frequency.
- **Goertzel** — single-bin magnitude equals the full-FFT bin.
- **Windowing** — coherent gain `Σw`, symmetry, endpoint values, main-lobe width / peak side-lobe level.
- **SignalDetectors** — true/false detection on labelled known signals (threshold behaviour).

### 2. Passive filters — `filters/passive/`
`Fir`, `Iir`, `BiquadCascade`, `CicFilter`, `MovingAverage`, `ExponentialMovingAverage`,
`MedianFilter`, `NotchCombFilter`, `SavitzkyGolayFilter`.

- **M2 frequency response** — magnitude at DC, cutoff (−3 dB), and Nyquist vs analytic `H(e^{jω})`;
  passband ripple; stopband attenuation. Drive steady-state sinusoids and compare RMS ratios.
- **M3 impulse/step** — FIR impulse response equals its coefficients; step steady-state = DC gain
  = `H(1)`; EMA/MovingAverage reach the input mean.
- **M4 stability (IIR/Biquad)** — poles inside the unit circle (pole radius < 1); bounded output over
  a long run; impulse response decays to zero.
- **M6 boundary** — disabled ⇒ pass-through; `Reset()` restores initial state; zero-in ⇒ zero-out.
- **Specialised** — Median rejects isolated impulses (order-statistic correctness); Savitzky-Golay
  reproduces polynomials up to its order exactly and estimates derivatives; CIC gain `= (R·M)^N`
  with expected pass-band droop; Notch/Comb null depth at target frequencies.

### 3. Stochastic / adaptive filters & online estimators — `filters/active/`, `estimators/online/`
`KalmanFilter`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter`, `KalmanSmoother`,
`ComplementaryFilter`, `AlphaBetaFilter`, `LmsAdaptiveFilter`, `RecursiveLeastSquares`.

- **M8 estimate error** — state estimate converges to ground truth on a simulated known system
  (RMSE below a bound).
- **M8 covariance consistency** — covariance stays symmetric positive-definite; normalised error
  (NEES) / innovation (NIS) within the χ² confidence band; innovations approximately white.
- **M4/M1 optimality** — steady-state Kalman gain matches the `DiscreteAlgebraicRiccatiEquation`
  solution for the linear-Gaussian case.
- **M5 convergence (LMS/RLS)** — error/MSE decreases monotonically toward the Wiener/LS solution;
  RLS matches the batch least-squares fit after processing all samples; forgetting factor behaves.
- **UKF** — sigma-point set recovers the mean and covariance of a known distribution.
- **Complementary/AlphaBeta** — bounded tracking lag; noise-reduction ratio vs raw signal.

### 4. Controllers — `controllers/`
`PidIncremental`, `BangBangHysteresis`, `LeadLagCompensator`, `Lqr`, `Lqg`,
`IntegralStateFeedbackLqi`, `Mpc`, `LuenbergerObserver`, `GainScheduledController`,
`Feedforward2Dof`, `SaturationRateLimiter`.

- **M3 transient response** — closed-loop step response: rise time, settling time, % overshoot,
  steady-state error. Integral action ⇒ zero steady-state error to a step.
- **M4 stability** — closed-loop poles/eigenvalues inside the unit circle; bounded state on a
  reference plant.
- **M6 saturation / anti-windup** — output stays within configured limits; no wind-up after
  prolonged saturation; `SaturationRateLimiter` honours magnitude and slew limits.
- **M1 gain optimality (LQR/LQG/LQI)** — feedback gain matches the Riccati-derived gain; observer
  poles at the designed locations (`LuenbergerObserver`).
- **MPC** — respects input/state constraints over the horizon; recovers the unconstrained LQR law
  when constraints are inactive.
- **BangBang** — switches on the hysteresis band edges; no chattering inside the band.
- **LeadLag / Feedforward** — DC gain and phase lead/lag at the design frequency (M2).

### 5. Control analysis — `control_analysis/`
`ControllabilityObservability`, `FrequencyResponse`, `RootLocus`.

- **M1 rank correctness** — controllable/observable flag and Gramian rank for hand-built
  controllable and deliberately uncontrollable systems.
- **M2 frequency response** — magnitude/phase at sample frequencies vs the analytic transfer
  function; correct gain and phase margins.
- **M4/M1 root locus** — branch points for a known plant: loci start at poles and end at zeros/∞;
  asymptote angles and breakaway points match closed-form values.

### 6. Offline estimators / regression — `estimators/offline/`
`LinearRegression`, `PolynomialFitting`, `YuleWalker`, `ExpectationMaximization`.

- **M1 exact fit** — noiseless data on a line/polynomial ⇒ coefficients recovered to tolerance,
  residual ≈ 0.
- **M8 statistical quality** — on noisy data: R²/RMSE within bounds; estimator unbiased across seeds;
  overdetermined fit equals the normal-equation solution.
- **YuleWalker** — recovers the AR coefficients of a known AR process; reflection coefficients
  `|k| < 1`.
- **EM (M5)** — log-likelihood increases monotonically each iteration; converges to known mixture
  parameters.
- **M9 conditioning** — behaviour on near-collinear features (ill-conditioned design matrix).

### 7. Optimization — `optimization/`
`GradientDescent`, `BayesianOptimization`.

- **M1/M5 convergence to optimum** — reaches the exact minimum of a quadratic; nears the minimum of
  a standard non-convex test function (e.g. Rosenbrock) within the iteration budget.
- **M5 monotonicity** — objective decreases each step for a suitable step size; diverges/oscillates
  for too-large steps (documented boundary).
- **M1 gradient check** — analytic gradient matches a finite-difference estimate.
- **Bayesian** — best-observed value improves over iterations and locates the optimum of a cheap
  known function within the budget.

### 8. Regularization — `regularization/`
`L1`, `L2`.

- **M1 closed form** — L2 shrinks a coefficient by `1/(1+λ)`; L1 soft-thresholds by `λ` (drives
  small coefficients to exactly zero).
- **M7 penalty value & gradient** — penalty equals `λ·‖w‖₁` / `λ·‖w‖₂²`; sub/gradient correct.
- **M6 boundary** — `λ = 0` ⇒ identity; large `λ` ⇒ coefficients → 0.

### 9. Solvers — `solvers/`
`GaussianElimination`, `CholeskyDecomposition`, `DiscreteAlgebraicRiccatiEquation`, `LevinsonDurbin`,
`DurandKerner`, `RungeKuttaIntegrators`, `DormandPrince45`, `OdeSystem`.

- **M1 residual** — linear solves: `‖A·x − b‖` below tolerance on a hand-solved system; Cholesky
  reconstructs `A = L·Lᵀ`; assert SPD precondition handling.
- **M4/M1 DARE** — solution `P` symmetric positive-definite and the Riccati residual ≈ 0; matches a
  known small-system solution.
- **LevinsonDurbin** — solution equals the direct Toeplitz solve; reflection coefficients `|k| < 1`.
- **DurandKerner (M5)** — each returned root satisfies `p(root) ≈ 0`; recovers known roots of a
  factored polynomial; converges within max iterations.
- **ODE integrators (M1/M5)** — reproduce analytic solutions (exponential decay, harmonic
  oscillator) to tolerance; **order of accuracy**: global error scales as `h^p` (assert the slope);
  exact for polynomials up to the method order; adaptive step (DormandPrince45) keeps local error
  within the requested tolerance.
- **M7 conservation** — energy drift bounded for a conservative system over many steps.

### 10. Neural network — `neural_network/`
`activation/*`, `layer/Dense`, `losses/*`, `model/Model`.

- **M1 activation values** — reference points: `sigmoid(0)=0.5`, `tanh(0)=0`, `relu(−x)=0`,
  `leaky_relu` slope; output range bounds; monotonicity where expected.
- **M7 softmax** — outputs sum to 1 and are non-negative; shift-invariance.
- **M1 loss values & gradients** — MSE of identical vectors = 0; loss non-negative; analytic gradient
  matches finite-difference (M1 gradient check).
- **M1 dense layer** — `output = W·x + b`; back-prop gradient check.
- **Model (M6)** — forward pass is deterministic and equals the manual layer composition.

### 11. Math foundation — `math/`
`Matrix`, `ComplexNumber`, `Quaternion`, `Cordic`, `TrigonometricFunctions`, `HyperbolicFunctions`,
`AdvancedFunctions`, `Statistics`, `LinearTimeInvariant`, `Toeplitz`, `QNumber`.

- **M1 accuracy vs `std::`** — trig/hyperbolic/CORDIC absolute (and where relevant ULP) error across
  the input range, including range-reduction boundaries.
- **M7 identities** — `sin²+cos² = 1`; `cosh²−sinh² = 1`; quaternion `‖q‖` preserved under
  multiplication; rotation composition; `q·q⁻¹ = 1`.
- **M1 matrix algebra** — `A·A⁻¹ = I` residual; determinant / transpose / multiply vs known results;
  associativity.
- **M9 conditioning / quantization** — `QNumber` quantization-error bound and saturation behaviour;
  `Statistics` numerically stable mean/variance (Welford) vs the naive formula.
- **LTI** — step/impulse response and pole/zero placement vs analytic (M3/M4).

---

## Anti-patterns

- Golden-output snapshots with no independent reference ("the output is whatever it printed").
- One test per parameter value instead of one per property (violates *no redundant tests*).
- Asserting only "no NaN / no crash" without a numerical reference.
- Heap-allocated signal buffers in tests — use `std::array` / bounded buffers.
- Re-deriving the algorithm inside the test as the "reference" — the reference must be independent
  (closed form, hand computation, or a distinct method).
