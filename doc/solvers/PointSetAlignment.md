# Point-Set Alignment (Kabsch / Umeyama)

## Overview & Motivation

Many estimation problems reduce to one question: given two sets of corresponding points, which rigid motion maps one onto the other? Scan matching asks it at every iteration of ICP. Calibration asks it to relate a camera to a robot flange or a motion-capture frame to a robot base. Fiducial and marker registration asks it to place a tracked object.

The **Kabsch algorithm** answers in closed form. It finds the rotation and translation that minimise the weighted sum of squared distances between the transformed source points and their targets. **Umeyama's** extension also estimates a uniform scale. In three dimensions the solution needs one $3 \times 3$ singular value decomposition, and in two dimensions a single arctangent.
Neither involves iteration or an initial guess.

## Mathematical Theory

### Problem

Given source points $p_i$, target points $q_i$ and non-negative weights $w_i$, find a proper rotation $R \in SO(d)$, a translation $t$ and optionally a scale $s > 0$ that minimise

$$E(R, t, s) = \sum_i w_i \left\| q_i - (s R p_i + t) \right\|^2$$

### Centring

With the total weight $W = \sum_i w_i$ and the weighted centroids $\mu_p$ and $\mu_q$, the optimal translation always maps $\mu_p$ onto $\mu_q$. The rotation follows from the centred points $a_i = p_i - \mu_p$ and $b_i = q_i - \mu_q$ through the cross-covariance

$$\Sigma = \frac{1}{W}\sum_i w_i\, b_i a_i^T, \qquad \sigma_p^2 = \frac{1}{W}\sum_i w_i \|a_i\|^2$$

Accumulating on centred coordinates avoids the single-precision cancellation of the textbook form $\sum p q^T - N \mu_p \mu_q^T$ when the clouds are far from the origin.

### Three Dimensions: Kabsch with the Reflection Fix

With the singular value decomposition $\Sigma = U\,\mathrm{diag}(d_0, d_1, d_2)\,V^T$, the optimal rotation is

$$R = U\,\mathrm{diag}\big(1, 1, \operatorname{sign}(\det U \det V)\big)\,V^T$$

The sign term prevents a reflection ($\det R = -1$), which the unconstrained solution $U V^T$ returns for mirrored, noisy or near-planar data. It flips the weakest axis instead. Collinear points leave the rotation about their line undetermined: $d_1$ vanishes, and the alignment is refused.

### Two Dimensions: Closed Form

In the plane, the optimal angle follows directly from the cross-covariance, with no decomposition:

$$\theta = \operatorname{atan2}\big(\Sigma_{10} - \Sigma_{01},\; \Sigma_{00} + \Sigma_{11}\big)$$

The result is always a proper rotation. Only coincident points are degenerate.

### Scale and Residual

Umeyama's scale is the trace of $R^T\Sigma$ divided by the source spread:

$$s = \frac{\operatorname{tr}(R^T \Sigma)}{\sigma_p^2}, \qquad t = \mu_q - s R \mu_p$$

The reported residual is the weighted root-mean-square distance after alignment. Weights of zero remove points, such as outliers rejected by an ICP iteration, without copying the point sets.

## Complexity Analysis

| Case             | Time   | Space  | Notes                                                        |
|------------------|--------|--------|--------------------------------------------------------------|
| Two dimensions   | $O(N)$ | $O(1)$ | Two passes over the points and one arctangent                |
| Three dimensions | $O(N)$ | $O(1)$ | Two passes and one $3 \times 3$ singular value decomposition |
| Residual         | $O(N)$ | $O(1)$ | One extra pass over the points                               |

## Step-by-Step Walkthrough

Four planar points $(0, 0)$, $(1, 0)$, $(0, 2)$, $(-1, 1)$ and five spatial points $(0, 0, 0)$, $(1, 0, 0)$, $(0, 2, 0)$, $(0, 0, 3)$, $(1, 1, 1)$:

1. **Planar motion**: rotating the planar points by $30°$ and shifting them by $(0.5, -1)$ is recovered exactly, with zero residual.
2. **Spatial motion**: a rotation of $0.7$ rad about $(1, 2, 3)/\sqrt{14}$ plus an offset $(0.2, -0.4, 1.0)$ is recovered to $10^{-5}$.
3. **Scale**: the same motion with the points scaled by $2.5$ gives $s = 2.5$ when scale estimation is on.
4. **Mirror**: matching the spatial points against their mirror image in the $z$ axis returns a proper rotation with residual $0.925$. A reflection would fit them exactly, which is exactly what the sign fix prevents.
5. **Outlier**: moving one target point by $(5, 5, 5)$ raises the residual to $3.44$. Giving that point zero weight recovers the exact motion.
6. **Noise**: adding noise of about $1$ cm to the planar targets gives the angle $0.5287$ rad, the translation $(0.508, -0.997)$ and a residual of $0.0165$.
7. **Degenerate inputs**: collinear spatial points, a single planar point, coincident planar points and mismatched sizes are all refused.

## Pitfalls & Edge Cases

- **Correspondences**: the algorithm assumes `target[i]` corresponds to `source[i]`. Wrong pairs give a wrong but confident answer. ICP alternates association and alignment for this reason.
- **Reflections**: dropping the sign fix lets mirrored or planar data produce an improper rotation.
- **Collinear data**: in three dimensions, points on a line leave one rotation axis free. The alignment is refused rather than returning an arbitrary rotation.
- **Scale on noisy data**: Umeyama's scale is biased low when the source points are noisy. Estimate scale only when it is genuinely unknown.
- **Weights**: at least as many positively weighted points as dimensions are required.

## Variants & Generalizations

- **Quaternion method (Horn)**: the same optimum from the largest eigenvector of a $4 \times 4$ matrix.
- **Point-to-plane and point-to-line metrics**: linearised alignment for surfaces and contours, used in ICP.
- **Robust alignment**: iteratively reweighted least squares with Huber or Tukey weights.
- **Hand-eye calibration**: solving $AX = XB$ with rotations aligned in the same spirit.

## Applications

- The alignment step of point-to-point ICP scan matching.
- Frame and hand-eye calibration from paired measurements.
- Fiducial marker, motion-capture and medical image registration.
- Shape analysis and structural biology (root-mean-square deviation of molecular structures).

## Connections to Other Algorithms

- [Singular Value Decomposition](SingularValueDecomposition.md): provides the rotation in three dimensions.
- [Levenberg–Marquardt](../optimization/LevenbergMarquardt.md): the iterative alternative for non-rigid or non-Euclidean residuals.

## References & Further Reading

- Kabsch, W. (1976). "A solution for the best rotation to relate two sets of vectors." *Acta Crystallographica*, A32, 922–923.
- Umeyama, S. (1991). "Least-squares estimation of transformation parameters between two point patterns." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 13(4), 376–380.
- Arun, K.S., Huang, T.S. and Blostein, S.D. (1987). "Least-squares fitting of two 3-D point sets." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 9(5), 698–700.
