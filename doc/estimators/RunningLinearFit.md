# Running Linear Fit

## Overview & Motivation

Many identification tasks reduce to fitting one straight line through a stream of points: a back-EMF constant from voltage against speed, a viscous-friction coefficient from torque against speed, a sensor gain and offset from readings against a reference.
The points arrive one at a time, often far more of them than fit in memory, and the slope is wanted together with an honest measure of how well it is determined.
A batch least-squares solve needs all points at once; a full recursive least-squares filter carries a covariance matrix and tuning for a problem that has only two unknowns.
The running linear fit keeps five numbers, updates them in constant time per point, and answers the slope, the intercept and the relative standard error of the slope at any moment.
Optional per-point weights and an exponential decay let it fold in pre-averaged runs and forget old data.

## Mathematical Theory

### Weighted sums of deviations

For points $(x_i, y_i)$ with weights $w_i$, the fit keeps the total weight $W$, the weighted means $\bar x, \bar y$ and the sums of deviations

$$S_{xx} = \sum_i w_i (x_i - \bar x)^2, \qquad S_{xy} = \sum_i w_i (x_i - \bar x)(y_i - \bar y), \qquad S_{yy} = \sum_i w_i (y_i - \bar y)^2.$$

### Update (Welford form)

Adding a point $(x, y)$ with weight $w$:

$$W' = W + w, \quad \bar x' = \bar x + \frac{w}{W'}(x - \bar x), \quad \bar y' = \bar y + \frac{w}{W'}(y - \bar y),$$

$$S'_{xx} = S_{xx} + w\,(x - \bar x)(x - \bar x'), \quad S'_{xy} = S_{xy} + w\,(x - \bar x)(y - \bar y'), \quad S'_{yy} = S_{yy} + w\,(y - \bar y)(y - \bar y').$$

Multiplying the deviation from the old mean by the deviation from the new one is what keeps the update numerically stable: it never forms the large, nearly cancelling sums $\sum x^2$ and $(\sum x)^2$.

### Decay

Exponential forgetting with factor $\lambda \in [0, 1]$ scales the weight and the three sums by $\lambda$ and leaves the means unchanged. The next point then moves the means by its share $w/(\lambda W + w)$, exactly as if every earlier weight had been multiplied by $\lambda$. $\lambda = 0$ forgets everything.

### Line and its uncertainty

The least-squares line $y = a x + b$ has

$$a = \frac{S_{xy}}{S_{xx}}, \qquad b = \bar y - a \bar x.$$

With unit weights and $n$ points, the residual sum of squares is $S_{yy} - a S_{xy}$, and the standard error of the slope is

$$\sigma_a = \sqrt{\frac{S_{yy} - a S_{xy}}{(n - 2)\, S_{xx}}}.$$

The fit reports $\sigma_a / |a|$, the relative standard error, which is scale-free and directly answers "is the slope determined to within one percent?".

## Complexity Analysis

| Operation    | Time   | Space  | Notes                                       |
|--------------|--------|--------|---------------------------------------------|
| Add          | $O(1)$ | $O(1)$ | Two divisions, a few multiply-adds          |
| Decay        | $O(1)$ | $O(1)$ | Four multiplications                        |
| Slope, error | $O(1)$ | $O(1)$ | One division; one square root for the error |

The state is the point count, the total weight, two means and three sums, independent of the number of points.

## Step-by-Step Walkthrough

Points $(1, 1)$, $(2, 3)$, $(3, 2)$ with unit weights:

| After  | $\bar x$ | $\bar y$ | $S_{xx}$ | $S_{xy}$ |
|--------|----------|----------|----------|----------|
| (1, 1) | 1        | 1        | 0        | 0        |
| (2, 3) | 1.5      | 2        | 0.5      | 1        |
| (3, 2) | 2        | 2        | 2        | 1        |

The slope is $1/2 = 0.5$ and the intercept $2 - 0.5 \times 2 = 1$. $S_{yy} = 2$, so the residual is $2 - 0.5 \times 1 = 1.5$ and $\sigma_a = \sqrt{1.5 / (1 \times 2)} \approx 0.87$, a relative error of 173 %: three points barely determine a slope.

## Pitfalls & Edge Cases

**No spread in $x$** — when every point has the same abscissa, $S_{xx} = 0$ and no slope exists; the fit reports a slope of zero and an infinite relative error. Check the spread of $x$, not only the number of points, before trusting a slope.

**Fewer than three points, or a zero slope** — the relative error is reported as infinite: two points fit any line exactly, and a relative error of a zero slope is undefined.

**Weights change the error estimate** — the standard error above assumes unit weights. With weights, the count still advances by one per call, so the error describes a fit of the call count, not of the summed weight; use it as a relative indicator.

**Decay keeps the means** — forgetting reduces the influence of old points but does not move the means; a sudden change in the operating point is followed only as new points arrive.

**Precision** — in single precision, accumulating many thousands of points with a large mean loses resolution in the sums; centre or rescale the inputs if the mean is far larger than the spread.

## Variants & Generalizations

[Recursive Least Squares](RecursiveLeastSquares.md) generalises the fit to many regressors with a covariance matrix. [Linear Regression](LinearRegression.md) solves the batch problem by QR. Total least squares treats noise in $x$ as well as $y$. Folding pre-averaged runs with their counts as weights reduces both memory and the influence of per-sample noise.

## Applications

- **Motor identification** — flux linkage from back-EMF against speed; viscous friction from steady-speed torque against speed, with forgetting for slowly drifting parameters.
- **Sensor calibration** — gain and offset of a transducer against a reference ramp.
- **Trend estimation** — the slope of a slowly varying signal with an estimate of its reliability.

## Connections to Other Algorithms

Welford's algorithm for the running variance is the one-variable case. [Recursive Least Squares](RecursiveLeastSquares.md) with two regressors $[x, 1]$ produces the same line with more state.

## References & Further Reading

- B. P. Welford, "Note on a method for calculating corrected sums of squares and products," *Technometrics* 4(3), 1962
- T. F. Chan, G. H. Golub, R. J. LeVeque, "Algorithms for computing the sample variance: analysis and recommendations," *The American Statistician* 37(3), 1983
