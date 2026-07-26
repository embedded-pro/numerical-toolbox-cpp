# Momentum-Based Collision-Detection Observer — Overview

## What it is
An online observer that estimates the **external torques** acting on a robot's joints — from a
collision, a human contact, or an unmodeled load — **without any joint-torque or acceleration
sensors**. It watches the robot's *generalized momentum* and flags the discrepancy between how the
momentum actually evolves and how the model says it should.

## Why it matters (embedded)
Safe human-robot interaction needs fast, cheap contact detection. Torque sensors on every joint
are expensive and fragile; numerically differentiating position twice to get acceleration is noisy.
The momentum observer needs only signals already available — position, velocity, and motor torque —
and runs in a few vector operations per cycle, making it ideal for real-time safety loops.

## How it works (intuition)
The generalized momentum `p = M(q)·q̇` changes according to the applied torque plus any external
torque. By integrating the *known* right-hand side of the momentum equation and comparing it with
the *measured* momentum, the observer forms a **residual** `r`. With no contact the residual sits
at zero; when an external torque appears, `r` tracks it as a first-order (low-pass) estimate whose
speed is set by the observer gain. Crucially the acceleration term cancels out, so no `q̈` is needed.

## Key parameters
- **Observer gain `K_O`** — sets the residual bandwidth: fast response vs. noise rejection.
- **Detection threshold** — per-joint residual level that declares a collision.
- **Model quality** — accurate `M, C, g` (and friction) keep the contact-free residual near zero.

## Reference
A. De Luca, A. Albu-Schäffer, S. Haddadin, G. Hirzinger, "Collision Detection and Safe Reaction
with the DLR-III Lightweight Manipulator Arm," *IEEE/RSJ IROS*, 2006.

## See also
`RecursiveNewtonEuler` (supplies `M, C, g`), `DynamicParameterIdentification` (accurate model
parameters), `DisturbanceObserver` (general disturbance estimation).
