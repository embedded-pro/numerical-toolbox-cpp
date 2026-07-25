# Model Reference Adaptive Control (MRAC) — Overview

## What it is
A self-tuning controller that makes an **uncertain** plant behave like a chosen **reference
model**. A designer picks the ideal response (a stable `A_m`, `B_m`); MRAC then adjusts its own
gains *online*, from the tracking error alone, until the real plant's output matches that ideal —
without ever measuring the unknown plant parameters directly.

## Why it matters (embedded)
Real hardware drifts: motor resistance rises with temperature, payload mass changes, actuators
age. A fixed controller tuned at the factory degrades. MRAC keeps performance constant by adapting
in the field — the same firmware handles a family of units and slowly-varying plants without
re-tuning. It is the classic direct-adaptive scheme for these "known structure, unknown numbers"
problems.

## How it works (intuition)
Run the reference model alongside the plant and watch the gap `e = x − x_m`. If the plant lags, the
adaptation law nudges the control gains in the direction that shrinks `e` — an online gradient
descent on the tracking error. Two flavours: the **MIT rule** follows the raw error gradient
(simple, but can go unstable if pushed hard), while the **Lyapunov redesign** derives the *same
shape* of update from a stability certificate, guaranteeing the error stays bounded. Feed it a
rich enough command and the gains also converge to their true ideal values.

## Key parameters
- **reference model (A_m, B_m)** — the injected "gold standard" behaviour to imitate.
- **adaptation gain γ** — how aggressively parameters move; the central speed-vs-stability knob.
- **sign of the input gain** — the adaptation must know which way the plant responds.
- **robustness modification** (σ / e-mod / projection) — optional, bounds parameter drift under
  noise and disturbance.

## Reference
K. J. Åström, B. Wittenmark, *Adaptive Control*, 2nd ed. (1995);
K. S. Narendra, A. M. Annaswamy, *Stable Adaptive Systems* (1989).

## See also
`RecursiveLeastSquares` (indirect-adaptive alternative: identify then control);
`FeedbackLinearization` / `BacksteppingControl` (fixed-parameter nonlinear designs MRAC augments);
`math::LinearTimeInvariant` (the reference-model container).
