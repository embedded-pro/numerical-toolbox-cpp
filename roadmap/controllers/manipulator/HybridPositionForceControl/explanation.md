# Hybrid Position/Force Control — Overview

## What it is
A task-space controller that splits the end-effector's directions into two disjoint groups: some
axes are **position-controlled**, the rest are **force-controlled**. A diagonal selection matrix
`S` decides which is which, and its complement `I−S` handles the others.

## Why it matters (embedded)
Many contact tasks are naturally hybrid: sliding a tool on a surface, you want to *track a path*
tangentially while *regulating the normal force* — you cannot command both position and force on the
same axis, because the environment already fixes one of them. Real-time force regulation with clean
axis partitioning is essential for deburring, polishing, assembly, and grinding on resource-limited
controllers.

## How it works (intuition)
Along motion axes a PD law pulls the tool toward the reference path. Along force axes a PI law drives
the measured contact force to the desired force. The two commands live in orthogonal subspaces, are
summed into a single Cartesian wrench, then mapped to joint torques by `Jᵀ`. Because `S` and `I−S`
never overlap, the loops do not fight each other.

## Key parameters
- **S (selection matrix)** — which task axes are motion (1) vs force (0), set in the constraint frame.
- **Kp, Kd** — motion-subspace position/velocity gains.
- **Kf, Ki** — force-subspace proportional/integral gains (integral removes steady force error).
- **dt** — sample period for the force integral (needs anti-windup).

## Reference
M. Raibert, J. Craig, "Hybrid Position/Force Control of Manipulators," *ASME J. Dyn. Sys. Meas.
Control*, 1981.

## See also
`ImpedanceControl` (#M17, compliant unified motion/force), `OperationalSpaceControl` (#M18,
task-space dynamics), `SpatialJacobian` (#M8, the `Jᵀ` wrench map).
