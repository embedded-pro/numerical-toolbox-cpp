# Friction Compensation (Coulomb + Viscous + Stribeck) — Overview

## What it is
A feedforward model of actuator friction — the sum of **Coulomb** (constant drag), **viscous**
(speed-proportional drag), and **Stribeck** (extra "stiction" that fades as the axis starts moving)
terms — evaluated from the axis velocity and added to a torque (or force) command to cancel real
friction.

## Why it matters (embedded)
Friction is the dominant un-modeled effect in geared drives, lead screws and robot joints: it causes
stick-slip, tracking lag, and steady-state error that pure PD/PID struggles to remove. A few
multiplies, one `exp`, and one `tanh` per axis — evaluated in the control loop — recover much of that
lost accuracy without a larger, slower feedback gain that would risk instability.

## How it works (intuition)
Plot friction torque against velocity and you get a characteristic curve: a tall "breakaway" spike
near zero speed (stiction), dropping into a Coulomb plateau, then rising linearly (viscous) as speed
increases. The model reproduces that curve. Because the ideal curve is discontinuous at zero
velocity (the `sign` flip), a smooth `tanh` boundary layer replaces the hard sign so the feedforward
command does not chatter as the axis reverses direction.

## Key parameters
- **F_c (Coulomb)** — constant drag magnitude once sliding.
- **F_s (stiction)** — higher breakaway level at near-zero speed (`F_s ≥ F_c`).
- **F_v (viscous)** — drag proportional to velocity.
- **v_s (Stribeck velocity)** — how quickly stiction fades into the Coulomb plateau.
- **ε (smoothing width)** — trades zero-crossing smoothness against compensation sharpness.

## Reference
B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit, "A survey of models, analysis tools and
compensation methods for the control of machines with friction," *Automatica*, 30(7), 1994.

## See also
`PidIncremental` and `Feedforward2Dof` (loops this term is added to); robotics-toolbox-cpp's PD +
gravity and computed-torque controllers, which add it to their joint torque command.
