# Feedforward / 2-DOF Controller — Overview

## What it is
A wrapper that adds a **feedforward path** alongside an existing feedback controller, creating a
two-degree-of-freedom (2-DOF) structure. One "degree" (feedforward) shapes how the system follows
the reference; the other (feedback) rejects errors and disturbances.

## Why it matters (embedded)
A single feedback loop must compromise between fast reference tracking and robust disturbance
rejection — pushing one degrades the other. The feedforward term is open-loop, so it improves
tracking **without touching the feedback gains or stability margins**. This is how motion
controllers hit aggressive trajectories while staying stable.

## How it works (intuition)
The feedforward block predicts the command needed to follow the reference (ideally an
inverse-plant model, or just a gain). The feedback block then only has to correct the small
residual error. Because the two paths add, they can be designed and tuned independently: pick the
feedback for stability/robustness first, then add feedforward for tracking performance.

## Key parameters
- **feedforward map** — reference-to-command model (gain, filter, or inverse plant), injected.
- **feedback law** — the existing error-driven controller (e.g. PID), injected.
- **output clamp** — actuator saturation applied to the combined command.

## Reference
K. J. Åström, R. M. Murray, *Feedback Systems* (2008), Ch. 12 (feedforward and 2-DOF design).

## See also
`PidIncremental` (a typical injected feedback law); `SaturationRateLimiter` (the output clamp);
`GainScheduledController` (varying the feedforward/feedback across operating points).
