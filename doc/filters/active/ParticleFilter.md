# Particle Filter (Bounded SIR, Systematic Resampling)

## Overview & Motivation

Kalman filters represent the state belief with one Gaussian. That works when the dynamics and measurements are close to linear and the belief has one mode. It fails when a robot could be in one of several similar corridors, when a target may have turned left or right, or when the measurement model is far from Gaussian.

A **particle filter** represents the belief with a set of samples (particles), each carrying a weight. Prediction moves every particle through the motion model with its own noise draw. A measurement reweights the particles by its likelihood. Resampling then replaces improbable particles with copies of probable ones.
The representation handles any dynamics, any likelihood and any number of modes, at a cost proportional to the number of particles.

This implementation is sized at compile time and allocates nothing. The caller supplies the motion model and the measurement likelihood as callables, and the random numbers as arguments, so a seeded run is bit-for-bit reproducible.

## Mathematical Theory

### Sequential Importance Resampling

The belief at time $k$ is $\{(x_k^{(i)}, w_k^{(i)})\}_{i=1}^N$ with $\sum_i w_k^{(i)} = 1$. One cycle is:

$$x_k^{(i)} \sim p(x_k \mid x_{k-1}^{(i)}, u_k), \qquad w_k^{(i)} \propto w_{k-1}^{(i)}\, p(z_k \mid x_k^{(i)})$$

Prediction samples the motion model, and the update multiplies each weight by the measurement likelihood.

### Log-Domain Update

Likelihoods of long measurement vectors underflow single precision quickly ($e^{-200} = 0$). The update therefore works with log-likelihoods $\ell_i$ and subtracts their maximum before exponentiating:

$$s_i = \log w^{(i)} + \ell_i, \qquad w^{(i)} \leftarrow \frac{e^{s_i - \max_j s_j}}{\sum_m e^{s_m - \max_j s_j}}$$

The best particle always gets an unnormalised weight of $1$, so normalisation never divides by zero. If every particle has $\ell = -\infty$, the measurement is impossible under the current belief. The update then reports failure and leaves the weights unchanged, so the caller can decide what to do.

### Effective Sample Size

When a few particles carry nearly all the weight, the rest are wasted computation. The effective sample size measures this:

$$N_{\text{eff}} = \frac{1}{\sum_i \left(w^{(i)}\right)^2}$$

It equals $N$ for uniform weights and $1$ when one particle holds everything.

### Systematic Resampling

Resampling draws $N$ new particles in proportion to the weights. Systematic resampling uses one uniform number $u \in [0, 1)$ and $N$ evenly spaced pointers into the cumulative weights:

$$p_k = \frac{u + k}{N}, \qquad k = 0, \ldots, N - 1$$

Each pointer selects the first particle whose cumulative weight reaches it. This has the lowest variance of the common schemes and costs one random number per resampling. Resampling every step impoverishes the sample set, so it is usually triggered only when $N_{\text{eff}}$ falls below a fraction of $N$, commonly one half.

## Complexity Analysis

| Operation      | Time   | Space          | Notes                                                          |
|----------------|--------|----------------|----------------------------------------------------------------|
| Predict        | $O(N)$ | $O(1)$         | One motion-model call per particle                             |
| Update         | $O(N)$ | $O(N)$         | One likelihood, one logarithm and one exponential per particle |
| Resample       | $O(N)$ | $O(N)$         | One pass over the cumulative weights                           |
| Particle store | —      | $O(N \cdot d)$ | Two particle buffers plus weights and scratch                  |

For $N$ particles of dimension $d$. A 3-state filter with 500 particles uses 16 KB in single precision.

## Step-by-Step Walkthrough

1. **Uniform prior**: 41 particles evenly spaced on $[-2, 2]$ start with weights $1/41$ and an effective sample size of 41.
2. **Measurement**: a Gaussian measurement $z = 0.5$ with $\sigma = 0.5$ moves the weighted mean to $0.498$ (slightly pulled towards zero by the truncated prior). The effective sample size drops to $17.7$, and the heaviest particle sits at $0.5$.
3. **Underflow**: subtracting $1000$ from every log-likelihood leaves the normalised weights unchanged. Without the log-domain update, every weight would underflow to zero.
4. **Second measurement**: $z = 0.7$ moves the mean to $0.600$, close to the average of the two measurements, and the effective sample size drops to $12.5$.
5. **Resampling**: four particles $0, 1, 2, 3$ with weights $0.1, 0.2, 0.3, 0.4$ have an effective sample size of $3.33$. With $u = 0.5$, the pointers $0.125, 0.375, 0.625, 0.875$ select particles $1, 2, 3, 3$, and the weights become uniform.
6. **Threshold**: with a threshold of half the particles (2), no resampling happens. With $0.9$ of them (3.6), it does.
7. **Impossible measurement**: a likelihood of $-\infty$ everywhere is rejected, and the weights stay as they were.

## Pitfalls & Edge Cases

- **Sample impoverishment**: resampling too often collapses the set onto a few states. Resample only when the effective sample size is low, and keep enough motion noise.
- **Overconfident likelihoods**: a likelihood that is too sharp kills most particles in one step. Add a floor, such as a uniform outlier term, or temper the likelihood.
- **Angles**: the component-wise mean fails for headings across $\pm\pi$. Compute a circular mean from the weights, or use the highest-weight particle.
- **Too few particles**: the required count grows quickly with the state dimension. Particle filters suit low-dimensional states.
- **Kidnapping**: once all particles are wrong, the filter cannot recover by itself. Inject a few random particles when the average likelihood drops.

## Variants & Generalizations

- **Auxiliary and regularised particle filters**: better proposals and jittered resampling against impoverishment.
- **Rao–Blackwellised filters**: particles for the nonlinear part and Kalman filters for the rest, as in FastSLAM.
- **KLD sampling**: the particle count adapts to the spread of the belief.
- **Unscented and extended Kalman filters**: Gaussian alternatives for unimodal, mildly nonlinear problems.

## Applications

- Monte Carlo localization of mobile robots in occupancy grids.
- Visual tracking of objects through occlusions.
- Terrain-aided navigation and fault diagnosis with discrete modes.

## Connections to Other Algorithms

- [Kalman Filter](KalmanFilter.md): the optimal linear-Gaussian special case.
- [Unscented Kalman Filter](UnscentedKalmanFilter.md): deterministic sigma points instead of random particles, for unimodal beliefs.
- [Residual-Form Kalman Update](ResidualKalmanUpdate.md): the Gaussian update for problems where a particle filter is too costly.

## References & Further Reading

- Gordon, N.J., Salmond, D.J. and Smith, A.F.M. (1993). "Novel approach to nonlinear/non-Gaussian Bayesian state estimation." *IEE Proceedings F*, 140(2), 107–113.
- Doucet, A. and Johansen, A.M. (2009). "A tutorial on particle filtering and smoothing: fifteen years later." *Handbook of Nonlinear Filtering*, Oxford University Press.
- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapter 4.
