# Disturbance Observer

## Overview & Motivation

Real plants never match their nominal models. External loads, friction, actuator nonlinearities, and parameter drift inject unmodeled energy into the loop. A Disturbance Observer (DOB) lumps all of these effects into a single **equivalent disturbance** signal, estimates it online, and subtracts it from the control input so the plant behaves as if it were the clean nominal model.

The key insight is that the DOB wraps around any existing controller without redesigning it. An engineer who has already tuned a PID or LQR for the nominal plant can bolt on a DOB and gain strong disturbance rejection without revisiting the nominal design. This makes DOBs especially attractive for embedded motion controllers — motor drives, precision stages, robotic joints — where the plant is moderately well-known but subject to load variations the nominal model ignores.

## Mathematical Theory

### Setup

Let the true discrete-time plant be $P(z)$ and the nominal model be $P_n(z)$. The control input seen by the true plant is $u_a = u + d$, where $u$ is the commanded input and $d$ is the lumped equivalent disturbance that captures model mismatch, external loads, and friction. The plant output is

$$y = P(z)\, u_a = P(z)(u + d).$$

### Disturbance Estimate

If $P_n^{-1}(z)$ is applied to $y$, it reconstructs the effective input that the nominal plant would have needed to produce that output:

$$P_n^{-1}(z)\, y \approx u + d \quad \text{(if } P \approx P_n\text{)}.$$

Subtracting the actual commanded input $u$ isolates the disturbance:

$$\hat{d} = P_n^{-1}(z)\, y - u.$$

### Q-Filter and Properness

The plant inverse $P_n^{-1}(z)$ is generally improper (more zeros than poles) and amplifies high-frequency measurement noise. A low-pass **Q-filter** $Q(z)$ is cascaded to make the combination $Q(z)\,P_n^{-1}(z)$ proper and bandwidth-limited:

$$\hat{d} = Q(z)\,P_n^{-1}(z)\, y - Q(z)\, u.$$

The filter $Q(z)$ must have relative degree at least equal to the relative degree of $P_n(z)$ so the realization does not differentiate. Unity DC gain, $Q(1) = 1$, is required for complete rejection of constant (step) disturbances.

### Control Law

The DOB corrects the nominal controller output $c$ by subtracting the estimate:

$$u = c - \hat{d}.$$

The closed-loop system then sees an effective plant of $P_n(z)$ inside the Q-filter bandwidth — the actual mismatch and disturbances are cancelled — and approaches the uncorrected nominal plant behaviour outside the bandwidth.

### Frequency-Domain Interpretation

Let $L(z) = Q(z)\,P_n^{-1}(z)\,P(z)$. The closed-loop sensitivity from disturbance $d$ to output $y$ is

$$S_d(z) = \frac{P(z)(1 - Q(z))}{1 + P(z)C(z)(1 - Q(z))}.$$

Inside the Q-filter passband ($Q \approx 1$): $S_d \approx 0$ — the disturbance is rejected.
Outside the passband ($Q \approx 0$): $S_d$ equals the nominal sensitivity — the DOB is transparent.

### Stability Robustness

Robust stability requires the complementary sensitivity of the inner DOB loop to satisfy

$$\left|Q(e^{j\omega})\,\Delta_m(e^{j\omega})\right| < 1 \quad \forall\, \omega,$$

where $\Delta_m = (P - P_n)/P_n$ is the relative model uncertainty. Widening $Q$ improves disturbance rejection but shrinks the robust-stability margin — this is the fundamental DOB trade-off.

## Complexity Analysis

| Operation   | Time                      | Space              | Notes                                          |
|-------------|---------------------------|--------------------|------------------------------------------------|
| Construct   | $O(N \cdot S^2)$          | $O(S^2 + N)$       | DC gain simulation, $S$ = StateSize, $N$ = 512 |
| Compute     | $O(N_{\rm in})$           | $O(1)$ extra       | Per-channel biquad filter pair                 |
| Reset       | $O(N_{\rm in})$           | $O(1)$ extra       | Clears filter states                           |

All storage is fixed-size; no heap allocation occurs at any point in the lifecycle.

## Step-by-Step Walkthrough

Consider a first-order discrete plant ($n=1$, $m=p=1$) with $a=0.9$, $b=0.1$, $c=1$, DC gain $= b/(1-a) = 1$, and a second-order Butterworth Q-filter at 20 Hz (sample rate 1 kHz).

**Steady-state with constant disturbance $d = 0.5$, nominal command $c = 0$:**

1. Plant output settles to $y_{ss} = P(1)\,d = 1 \cdot 0.5 = 0.5$.
2. Q-filter path 1: $Q(1)\,P_n^{-1}(1)\,y_{ss} = 1 \cdot 1 \cdot 0.5 = 0.5$.
3. Q-filter path 2: $Q(1)\,u_{ss} = 1 \cdot (c - \hat{d})_{ss}$.
4. At equilibrium path 1 $-$ path 2 $= \hat{d}$ and $u_{ss} = c - \hat{d}$, giving $\hat{d} = 0.5 = d$. The estimate converges exactly.
5. The corrected input is $u = 0 - 0.5 = -0.5$, so the effective input to the plant is $-0.5 + 0.5 = 0$ — the disturbance is cancelled.

## Pitfalls & Edge Cases

- **Non-minimum-phase plants**: $P_n^{-1}(z)$ has unstable poles when $P_n$ has zeros outside the unit circle. The DOB inner loop becomes unstable; non-minimum-phase zeros must be treated specially or the DOB must not be applied directly.
- **DC gain of zero**: if the nominal plant has no steady-state response to the input, the inverse gain is ill-defined. The implementation guards against division by zero but the DOB will not function correctly.
- **Wide Q bandwidth**: increasing the cutoff trades rejection bandwidth for noise amplification and reduced robustness to model mismatch. The trade-off is captured by the robust-stability bound above.
- **Large model mismatch**: when $|\Delta_m|$ is not small, the DOB may amplify rather than cancel the disturbance. The bandwidth of $Q$ must be restricted so the robustness condition holds across the frequency range of significant mismatch.
- **Unstable nominal plant**: the steady-state DC-gain simulation used during construction diverges; only stable nominal plants are supported.

## Variants & Generalizations

- **Two-degree-of-freedom DOB**: a separate reference pre-filter shapes the tracking response independently of the disturbance rejection channel.
- **Nonlinear DOB**: replaces the linear inverse with a nonlinear observer (e.g., extended high-gain observer) for plants with known nonlinear structure.
- **Time-varying Q**: adapts the Q-filter bandwidth online to balance rejection versus robustness as operating conditions change.
- **Multi-input multi-output (MIMO) DOB**: generalises the scalar channel-pairing to full matrix $P_n^{-1}$, requiring the nominal plant to be square and invertible.

## Applications

- Precision motion control: rejects cutting forces in CNC machines and friction in ball-screw drives.
- Robotic joint torque control: cancels gravity, Coriolis, and friction terms without explicit model inversion.
- Hard-disk drive servo: one of the earliest industrial applications; Q-filter bandwidth sets the track-following bandwidth.
- Power electronics: rejects grid-voltage disturbances in inverter current control.

## Connections to Other Algorithms

- **Sliding Mode Control**: an alternative approach to matched-disturbance rejection via a switching term; SMC is discontinuous and model-free while the DOB is smooth and model-based.
- **Luenberger Observer / Kalman Filter**: estimate state from measurements; the DOB estimates disturbance from input-output pairs without augmenting the state.
- **Active Disturbance Rejection Control (ADRC)**: treats total disturbance as an augmented state in a full observer; conceptually similar to DOB but parameterised through observer bandwidth rather than a Q-filter.
- **BiquadCascade**: the Q-filter is realised directly as a second-order IIR section.

## References & Further Reading

- W.-H. Chen, J. Yang, L. Guo, S. Li, "Disturbance-Observer-Based Control and Related Methods — An Overview," *IEEE Transactions on Industrial Electronics*, 63(2), pp. 1083–1095, 2016.
- K. Ohishi, M. Nakao, K. Ohnishi, K. Miyachi, "Microprocessor-Controlled DC Motor for Load-Insensitive Position Servo System," *IEEE Transactions on Industrial Electronics*, 34(1), pp. 44–49, 1987.
- E. Schrijver, J. van Dijk, "Disturbance Observers for Rigid Mechanical Systems: Equivalence, Stability, and Design," *ASME Journal of Dynamic Systems, Measurement, and Control*, 124(4), pp. 539–548, 2002.
- S. Komada, K. Ohnishi, "Force Feedback Control of Robot Manipulator by the Acceleration Tracing Orientation Method," *IEEE Transactions on Industrial Electronics*, 37(1), pp. 6–12, 1990.
