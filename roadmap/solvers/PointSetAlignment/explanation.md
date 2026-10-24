# Point-Set Alignment (Kabsch / Umeyama) — Overview

## What it is
Given two sets of matching points — the same landmarks seen in two frames, a scan and a map, a
calibration target in two poses — it finds the rotation, translation and, optionally, the uniform scale
that best overlays one set on the other in the least-squares sense.

## Why it matters (embedded)
This is the core step of scan matching (ICP), sensor-to-sensor and hand-eye calibration, and fiducial
or motion-capture registration. The solution is closed-form: one small singular-value decomposition in
3-D, a single arctangent in 2-D. It runs in a bounded time with no iteration and no memory beyond a
few 3×3 matrices.

## How it works (intuition)
First both point sets are shifted so their weighted centres coincide; the remaining mismatch is pure
rotation (and scale). The cross-covariance matrix of the centred sets says how directions in one set
correlate with directions in the other, and its singular-value decomposition gives the rotation that
maximises this agreement. If the best orthogonal match happens to be a mirror image, the weakest axis is
flipped so the result stays a real rotation. The scale is the ratio of the matched spread to the source
spread, and the translation moves the rotated source centre onto the target centre.

## Key parameters
- **correspondences** — `target[i]` matches `source[i]`.
- **weights** — per-point confidence (0 drops a point).
- **estimate scale** — rigid (Kabsch) or similarity (Umeyama) fit.

## Reference
W. Kabsch, "A solution for the best rotation to relate two sets of vectors," *Acta Crystallographica*
A32, 1976; S. Umeyama, "Least-squares estimation of transformation parameters between two point
patterns," *IEEE T-PAMI*, 13(4), 1991.

## See also
`SingularValueDecomposition` (the 3-D solve), `SE3Transform` / `SE2Transform` (items 53, 66: wrap the result);
robotics-toolbox-cpp's ICP scan matching (E5) and hand-eye calibration (C1).
