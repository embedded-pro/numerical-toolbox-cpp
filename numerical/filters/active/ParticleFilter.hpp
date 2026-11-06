#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <limits>
#include <span>
#include <type_traits>

namespace filters
{
    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    class ParticleFilter
    {
        static_assert(std::is_floating_point_v<T>, "ParticleFilter supports floating-point types");
        static_assert(StateSize > 0, "ParticleFilter needs a state");
        static_assert(NumParticles >= 2, "ParticleFilter needs at least two particles");

    public:
        using StateVector = math::Vector<T, StateSize>;

        ParticleFilter();

        template<class Draw>
        void Initialize(Draw&& draw);
        template<class Propagate>
        OPTIMIZE_FOR_SPEED void Predict(Propagate&& propagate);
        template<class LogLikelihood>
        OPTIMIZE_FOR_SPEED bool Update(LogLikelihood&& logLikelihood);

        T EffectiveSampleSize() const;
        OPTIMIZE_FOR_SPEED void Resample(T u);
        bool ResampleIfNeeded(T essFraction, T u);

        StateVector Mean() const;
        const StateVector& MaximumWeight() const;
        std::span<const StateVector> Particles() const;
        std::span<StateVector> Particles();
        std::span<const T> Weights() const;

    private:
        static constexpr T uniformWeight{ T{ 1 } / static_cast<T>(NumParticles) };

        std::array<std::array<StateVector, NumParticles>, 2> buffers{};
        std::size_t active{ 0 };
        std::array<T, NumParticles> weights{};
        std::array<T, NumParticles> scratch{};
    };

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    ParticleFilter<T, StateSize, NumParticles>::ParticleFilter()
    {
        weights.fill(uniformWeight);
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    template<class Draw>
    void ParticleFilter<T, StateSize, NumParticles>::Initialize(Draw&& draw)
    {
        auto& particles{ buffers[active] };
        for (std::size_t i = 0; i < NumParticles; ++i)
            particles[i] = draw(i);
        weights.fill(uniformWeight);
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    template<class Propagate>
    OPTIMIZE_FOR_SPEED void ParticleFilter<T, StateSize, NumParticles>::Predict(Propagate&& propagate)
    {
        auto& particles{ buffers[active] };
        for (std::size_t i = 0; i < NumParticles; ++i)
            propagate(particles[i], i);
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    template<class LogLikelihood>
    OPTIMIZE_FOR_SPEED bool ParticleFilter<T, StateSize, NumParticles>::Update(LogLikelihood&& logLikelihood)
    {
        const auto& particles{ buffers[active] };
        T maximum{ -std::numeric_limits<T>::infinity() };
        for (std::size_t i = 0; i < NumParticles; ++i)
        {
            scratch[i] = math::Log(weights[i]) + logLikelihood(particles[i]);
            maximum = scratch[i] > maximum ? scratch[i] : maximum;
        }

        if (maximum == -std::numeric_limits<T>::infinity())
            return false;

        T sum{};
        for (std::size_t i = 0; i < NumParticles; ++i)
        {
            weights[i] = math::Exp(scratch[i] - maximum);
            sum += weights[i];
        }

        for (auto& weight : weights)
            weight /= sum;
        return true;
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    T ParticleFilter<T, StateSize, NumParticles>::EffectiveSampleSize() const
    {
        T sumOfSquares{};
        for (const auto weight : weights)
            sumOfSquares += weight * weight;
        return T{ 1 } / sumOfSquares;
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    OPTIMIZE_FOR_SPEED void ParticleFilter<T, StateSize, NumParticles>::Resample(T u)
    {
        const auto& source{ buffers[active] };
        auto& target{ buffers[1 - active] };
        T cumulative{ weights[0] };
        std::size_t index{ 0 };
        for (std::size_t k = 0; k < NumParticles; ++k)
        {
            const T pointer{ (u + static_cast<T>(k)) / static_cast<T>(NumParticles) };
            while (pointer > cumulative && index < NumParticles - 1)
            {
                ++index;
                cumulative += weights[index];
            }
            target[k] = source[index];
        }

        active = 1 - active;
        weights.fill(uniformWeight);
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    bool ParticleFilter<T, StateSize, NumParticles>::ResampleIfNeeded(T essFraction, T u)
    {
        if (EffectiveSampleSize() >= essFraction * static_cast<T>(NumParticles))
            return false;

        Resample(u);
        return true;
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    auto ParticleFilter<T, StateSize, NumParticles>::Mean() const -> StateVector
    {
        const auto& particles{ buffers[active] };
        StateVector mean{};
        for (std::size_t i = 0; i < NumParticles; ++i)
            mean = mean + particles[i] * weights[i];
        return mean;
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    auto ParticleFilter<T, StateSize, NumParticles>::MaximumWeight() const -> const StateVector&
    {
        std::size_t best{ 0 };
        for (std::size_t i = 1; i < NumParticles; ++i)
            best = weights[i] > weights[best] ? i : best;
        return buffers[active][best];
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    auto ParticleFilter<T, StateSize, NumParticles>::Particles() const -> std::span<const StateVector>
    {
        return buffers[active];
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    auto ParticleFilter<T, StateSize, NumParticles>::Particles() -> std::span<StateVector>
    {
        return buffers[active];
    }

    template<typename T, std::size_t StateSize, std::size_t NumParticles>
    std::span<const T> ParticleFilter<T, StateSize, NumParticles>::Weights() const
    {
        return weights;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ParticleFilter<float, 3, 64>;
    extern template class ParticleFilter<float, 1, 41>;
    extern template class ParticleFilter<float, 1, 4>;
    extern template void ParticleFilter<float, 1, 41>::Initialize<math::Vector<float, 1> (&)(std::size_t)>(math::Vector<float, 1> (&)(std::size_t));
    extern template void ParticleFilter<float, 1, 4>::Initialize<math::Vector<float, 1> (&)(std::size_t)>(math::Vector<float, 1> (&)(std::size_t));
    extern template bool ParticleFilter<float, 1, 41>::Update<float (&)(const math::Vector<float, 1>&)>(float (&)(const math::Vector<float, 1>&));
    extern template bool ParticleFilter<float, 1, 4>::Update<float (&)(const math::Vector<float, 1>&)>(float (&)(const math::Vector<float, 1>&));
    extern template void ParticleFilter<float, 1, 4>::Predict<void (&)(math::Vector<float, 1>&, std::size_t)>(void (&)(math::Vector<float, 1>&, std::size_t));
#endif
}
