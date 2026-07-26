# Active Disturbance Rejection Control (ADRC + ESO) — Overview

## What it is
A near model-free controller built around an **Extended State Observer (ESO)**. The ESO treats the
*total* disturbance — everything acting on the plant beyond a known input gain — as one extra state,
estimates it in real time, and a simple feedback law cancels it. What remains behaves like a clean
chain of integrators that a textbook PD loop can control.

## Why it matters (embedded)
ADRC delivers strong, robust motion control **without an accurate model**. You do not identify the
plant; you estimate and cancel its dynamics online. That is a huge win on microcontrollers driving
motors and actuators whose parameters drift with temperature, load, and wear — it is increasingly
the default in industrial drives precisely because it survives a bad model.

## How it works (intuition)
Write the plant as "a chain of integrators plus an unknown lump `f`, driven by `b0·u`." The ESO runs
this model, compares its predicted output to the measurement, and uses the error to correct **every**
state — including the extra state that stands in for `f`. Because `f` absorbs all the unmodeled
physics, once you subtract the estimate `f̂` in the control law the residual system is just the
integrator chain. **Bandwidth parameterization** then reduces all tuning to two intuitive dials:
how fast the observer watches (`ω_o`) and how fast the loop responds (`ω_c`).

## Key parameters
- **`ω_o` (observer bandwidth)** — how quickly the ESO tracks the disturbance; higher = faster but noisier.
- **`ω_c` (control bandwidth)** — closed-loop response speed after cancellation.
- **`b0` (input-gain estimate)** — the one plant number you must roughly know; the ESO forgives the rest.

## Reference
J. Han, "From PID to Active Disturbance Rejection Control," *IEEE Trans. Industrial Electronics*,
56(3), 2009; Z. Gao, "Scaling and Bandwidth-Parameterization Based Controller Tuning," *ACC*, 2003.

## See also
`LuenbergerObserver` (the linear-observer building block); `DisturbanceObserver` (transfer-function
sibling); `Pid` (the classical loop ADRC generalizes).
