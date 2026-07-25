# Transfer-Function ↔ State-Space Conversion — Overview

## What it is
A translator between the two dominant ways of writing a linear system: the classical **transfer
function** (a ratio of polynomials in `s` or `z`) and the modern **state-space** quadruple
`(A, B, C, D)`. It builds the *controllable* and *observable* canonical realisations from polynomial
coefficients, and runs the reverse trip — recovering the transfer function from a state-space model.

## Why it matters (embedded)
Classical design tools (Bode plots, root locus, PID/lead-lag tuning) speak transfer functions; modern
tools (LQR, observers, Kalman filters) speak state space. This converter is the bridge: design a filter
or compensator in the frequency domain, then drop it straight into the library's `LinearTimeInvariant`
runtime for execution on the target — or take a state-space plant and analyse its frequency response.
No re-derivation by hand, no transcription errors.

## How it works (intuition)
A monic denominator `sⁿ + a₁sⁿ⁻¹ + … + aₙ` maps *directly* into a **companion matrix**: put ones on the
super-diagonal and the negated coefficients `−aₙ … −a₁` along the bottom row. The numerator coefficients
become the output row `C`. That is the controllable canonical form; the observable form is simply its
transpose (the two are duals, so one build serves both). Going back the other way, the denominator is the
**characteristic polynomial** of `A`, and the Faddeev–Le Verrier recursion produces both it and the
numerator adjugate in one sweep. When the numerator degree equals the denominator degree, a single
polynomial division peels off the direct feed-through term `D`.

## Key parameters
- **numerator / denominator coefficients** — the transfer function; the denominator is normalised monic.
- **form choice** — controllable vs observable canonical realisation (transpose-related).
- **feed-through `D`** — nonzero only for proper (non-strictly-proper) transfer functions.

## Reference
T. Kailath, *Linear Systems*, Prentice-Hall, 1980 — canonical realisations and the state-space /
transfer-function correspondence.

## See also
`LinearTimeInvariant` (the realisation target); `ControllabilityObservability` (a realisation is minimal
iff controllable **and** observable); `FrequencyResponse` and `RootLocus` (classical analyses that consume
the transfer function); `DurandKerner` (factor the denominator into poles).
