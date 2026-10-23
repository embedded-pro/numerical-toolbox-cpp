# Damped Pseudo-Inverse and Null-Space Projector — Overview

## What it is
Two tools from one singular value decomposition: a "best effort" inverse of any matrix that stays
well-behaved when the matrix is nearly singular, and a projector that removes from any vector every
component the matrix can see.

## Why it matters (embedded)
Mapping desired outputs to inputs through a non-square or ill-conditioned matrix is everywhere: joint
rates from a tool velocity, motor commands from a desired thrust and torque, calibration corrections from
residuals. The plain pseudo-inverse explodes near rank loss and commands huge, saturating inputs; damping
caps that at a small, controlled accuracy cost. When there are more inputs than outputs, the projector
exposes the spare freedom so a second goal (stay away from limits, share load, save energy) can use it
without disturbing the first.

## How it works (intuition)
The SVD splits the matrix into independent directions, each with a gain. Inverting means dividing by each
gain; damping replaces `1/σ` by `σ/(σ² + λ²)`, which matches `1/σ` for large gains and fades to zero for
tiny ones instead of blowing up. The input directions with (numerically) zero gain form the null space;
subtracting a vector's components along the other input directions leaves only what the matrix cannot
feel. That projector must use the undamped, thresholded split — a damped one leaks into the primary task.

## Key parameters
- **Damping λ** — robustness near singularity versus accuracy (`error ≈ λ²/σ_min²`).
- **Rank tolerance** — relative threshold below which a direction counts as lost.

## Reference
C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped
least-squares methods," *IEEE Trans. SMC*, 16(1), 1986; A. Liégeois, *IEEE Trans. SMC*, 7(12), 1977;
G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., §6.1.

## See also
`SingularValueDecomposition` (the factorization), `LevenbergMarquardt` (#64, uses the same damping per
iteration); robotics-toolbox-cpp's redundancy resolution, a thin task-space wrapper over this.
