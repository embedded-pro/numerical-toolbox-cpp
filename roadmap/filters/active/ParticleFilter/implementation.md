# Particle Filter (Bounded SIR, Systematic Resampling) — Implementation Pseudocode

> Roadmap ref: #70 (Tier 3) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

Sequential importance resampling with a compile-time particle count. The caller supplies the motion
model (including its noise draws) and the measurement log-likelihood as callables. The filter keeps
normalised weights, measures degeneracy with the effective sample size, and resamples systematically
from a caller-provided uniform number. With no allocation and no internal random generator, a seeded
run is bit-for-bit reproducible.

## Data structures

```cpp
template<typename T, std::size_t StateSize, std::size_t NumParticles>   # static_assert(std::is_floating_point_v<T>); NumParticles ≥ 2
class ParticleFilter:
    using StateVector = math::Vector<T, StateSize>
    std::array<StateVector, NumParticles> buffers[2]      # current and resampling target
    std::size_t                           active{ 0 }
    std::array<T, NumParticles>           weights         # normalised, Σ = 1
    std::array<T, NumParticles>           scratch         # log-domain work area
```

## Interface

```cpp
template<class Draw>        void Initialize(Draw&& draw)                   # particles[i] = draw(i); weights = 1/N
template<class Propagate>   void Predict(Propagate&& propagate)            # propagate(particle&, i) in place; hot path
template<class LogLikelihood> bool Update(LogLikelihood&& logLikelihood)   # w ∝ w·exp(ℓ(x)); false if every ℓ = −∞
T    EffectiveSampleSize() const                                           # 1 / Σ wᵢ²
void Resample(T u)                                                         # systematic, u ∈ [0, 1); weights → 1/N
bool ResampleIfNeeded(T essFraction, T u)                                  # resample when ESS < essFraction·N
StateVector        Mean() const                                            # Σ wᵢ xᵢ, component-wise
const StateVector& MaximumWeight() const                                   # highest-weight particle
std::span<const StateVector> Particles() const;  std::span<StateVector> Particles()
std::span<const T>           Weights() const
```

Callables are template parameters, not `infra::Function`, so captured state of any size is allowed and
the calls inline.

## Algorithm (pseudocode)

```text
function Update(ℓ):                              # OPTIMIZE_FOR_SPEED; log domain against under/overflow
    for i: scratch[i] = log(weights[i]) + ℓ(xᵢ)  # log 0 = −∞ keeps dead particles dead
    m = max_i scratch[i]
    if m == −∞: return false                    # every particle impossible: weights unchanged, caller decides
    for i: weights[i] = exp(scratch[i] − m)
    normalise weights to Σ = 1
    return true

function EffectiveSampleSize():  return 1 / Σ weights[i]²

function Resample(u):                            # systematic: one uniform, N evenly spaced pointers
    c = weights[0];  j = 0
    for k in [0, N):
        target = (u + k) / N
        while target > c and j < N − 1: j += 1;  c += weights[j]
        buffers[1 − active][k] = buffers[active][j]
    active = 1 − active;  weights = 1/N

function ResampleIfNeeded(f, u):
    if EffectiveSampleSize() < f·N: Resample(u); return true
    return false
```

## Complexity & memory

- `Predict` / `Update`: `O(N)` callable invocations; `Update` adds `N` `exp` and `N` `log`. `Resample`: `O(N)`.
- Memory: `2·N·StateSize + 2·N` scalars plus an index. A 3-state, 500-particle filter uses 16 KB in
  `float`. No heap.

## Numerical / embedded notes

- **Log domain.** Likelihoods of long scans underflow `float` (`exp(−200) = 0`). Subtracting the maximum
  log-weight keeps the best particle at weight 1 before normalisation, so the update never divides by 0.
- **Systematic resampling** has the lowest variance of the common schemes and costs one random number
  per resampling, injected as `u` so the caller owns the generator (seeded, deterministic tests).
- **Resample only when needed.** Resampling every step adds sample impoverishment. `ResampleIfNeeded(0.5, u)`
  is the usual ESS rule (Doucet & Johansen, 2009).
- `Mean` is component-wise. For angles (headings in localization) the caller computes the circular mean
  from `Weights()`; `MaximumWeight` gives the MAP-style point estimate.
- Particles are mutable through `Particles()`, so the caller can inject random particles for
  kidnapped-robot recovery (augmented MCL).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/filters/active/ParticleFilter.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Predict`/`Update`/`Resample`, and `extern template class ParticleFilter<float, 3, 64>;` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD` (the template members are instantiated by the tests).
- Coverage: `numerical/filters/active/ParticleFilter.cpp` → `template class ParticleFilter<float, 3, 64>;`
- Test: `numerical/filters/active/test/TestParticleFilter.cpp`
- Doc: `doc/filters/active/ParticleFilter.md` (per `doc/TEMPLATE.md`) + row in `doc/filters/README.md`
- CMake: `.hpp` → `target_sources(numerical.filters.active PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.filters.active ...)`; test → `numerical.filters.active_test`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
