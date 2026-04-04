#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/optimization/BlackBoxObjective.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

namespace optimization
{
    struct GpHyperparameters
    {
        float lengthScale = 1.0f;
        float signalVariance = 1.0f;
        float noiseVariance = 1e-2f;
    };

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates = 100>
    class BayesianOptimization
    {
        static_assert(NumParams > 0, "NumParams must be positive");
        static_assert(MaxObservations >= 2, "MaxObservations must be at least 2");
        static_assert(NumCandidates > 0, "NumCandidates must be positive");

    public:
        using ParameterVector = math::Vector<float, NumParams>;
        using BoundsArray = std::array<std::pair<float, float>, NumParams>;

        struct Result
        {
            ParameterVector bestPoint;
            float bestValue;
            std::size_t evaluations;
        };

        BayesianOptimization(const BoundsArray& bounds,
            const GpHyperparameters& hyperparameters,
            uint64_t seed = 42ULL);

        void AddObservation(const ParameterVector& point, float value);

        [[nodiscard]] std::pair<float, float> GpPredict(const ParameterVector& query) const;

        [[nodiscard]] float ExpectedImprovement(const ParameterVector& query, float bestValue) const;

        OPTIMIZE_FOR_SPEED ParameterVector MaximizeAcquisition(float bestValue);

        OPTIMIZE_FOR_SPEED Result Optimize(BlackBoxObjective<NumParams>& objective,
            std::size_t numIterations);

        [[nodiscard]] const std::array<float, MaxObservations>& GetObservedValues() const;
        [[nodiscard]] std::size_t GetNumObservations() const;

    private:
        [[nodiscard]] static float SquaredExpKernel(const ParameterVector& a,
            const ParameterVector& b,
            const GpHyperparameters& hp);

        void RebuildKernelMatrix();

        OPTIMIZE_FOR_SPEED ParameterVector LcgSample();

        [[nodiscard]] static float NormalCdf(float z);
        [[nodiscard]] static float NormalPdf(float z);

        BoundsArray bounds_;
        GpHyperparameters hyperparameters_;
        uint64_t lcgState_;
        std::size_t numObservations_{ 0 };

        std::array<ParameterVector, MaxObservations> observedPoints_{};
        std::array<float, MaxObservations> observedValues_{};
        math::SquareMatrix<float, MaxObservations> kernelMatrix_{};
        math::Vector<float, MaxObservations> alpha_{};
    };

    // Implementation //

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    BayesianOptimization<NumParams, MaxObservations, NumCandidates>::BayesianOptimization(
        const BoundsArray& bounds,
        const GpHyperparameters& hyperparameters,
        uint64_t seed)
        : bounds_(bounds)
        , hyperparameters_(hyperparameters)
        , lcgState_(seed)
    {
        for (std::size_t d = 0; d < NumParams; ++d)
            really_assert(bounds_[d].first < bounds_[d].second);

        kernelMatrix_ = math::SquareMatrix<float, MaxObservations>::Identity();
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    void BayesianOptimization<NumParams, MaxObservations, NumCandidates>::AddObservation(
        const ParameterVector& point, float value)
    {
        really_assert(numObservations_ < MaxObservations);
        observedPoints_[numObservations_] = point;
        observedValues_[numObservations_] = value;
        ++numObservations_;
        RebuildKernelMatrix();
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    void BayesianOptimization<NumParams, MaxObservations, NumCandidates>::RebuildKernelMatrix()
    {
        for (std::size_t i = 0; i < MaxObservations; ++i)
        {
            for (std::size_t j = 0; j < MaxObservations; ++j)
            {
                if (i < numObservations_ && j < numObservations_)
                {
                    float k = SquaredExpKernel(observedPoints_[i], observedPoints_[j], hyperparameters_);
                    if (i == j)
                        k += hyperparameters_.noiseVariance * hyperparameters_.noiseVariance;
                    kernelMatrix_.at(i, j) = k;
                }
                else
                {
                    kernelMatrix_.at(i, j) = (i == j) ? 1.0f : 0.0f;
                }
            }
        }

        math::Vector<float, MaxObservations> yPadded{};
        for (std::size_t i = 0; i < numObservations_; ++i)
            yPadded.at(i, 0) = observedValues_[i];

        alpha_ = solvers::SolveSystem<float, MaxObservations, 1>(kernelMatrix_, yPadded);
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    std::pair<float, float>
    BayesianOptimization<NumParams, MaxObservations, NumCandidates>::GpPredict(
        const ParameterVector& query) const
    {
        math::Vector<float, MaxObservations> kStar{};
        for (std::size_t i = 0; i < numObservations_; ++i)
            kStar.at(i, 0) = SquaredExpKernel(query, observedPoints_[i], hyperparameters_);

        float mu = 0.0f;
        for (std::size_t i = 0; i < MaxObservations; ++i)
            mu += kStar.at(i, 0) * alpha_.at(i, 0);

        const auto v = solvers::SolveSystem<float, MaxObservations, 1>(kernelMatrix_, kStar);

        const float kSS = hyperparameters_.signalVariance * hyperparameters_.signalVariance;
        float vDotKStar = 0.0f;
        for (std::size_t i = 0; i < MaxObservations; ++i)
            vDotKStar += v.at(i, 0) * kStar.at(i, 0);

        const float sigma2 = std::max(0.0f, kSS - vDotKStar);
        return { mu, std::sqrt(sigma2) };
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    float BayesianOptimization<NumParams, MaxObservations, NumCandidates>::ExpectedImprovement(
        const ParameterVector& query, float bestValue) const
    {
        if (numObservations_ == 0)
            return 0.0f;

        const auto [mu, sigma] = GpPredict(query);

        if (sigma <= 0.0f)
            return 0.0f;

        const float z = (bestValue - mu) / sigma;
        return (bestValue - mu) * NormalCdf(z) + sigma * NormalPdf(z);
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    OPTIMIZE_FOR_SPEED
        typename BayesianOptimization<NumParams, MaxObservations, NumCandidates>::ParameterVector
        BayesianOptimization<NumParams, MaxObservations, NumCandidates>::MaximizeAcquisition(float bestValue)
    {
        ParameterVector bestCandidate = LcgSample();
        float bestEI = ExpectedImprovement(bestCandidate, bestValue);

        for (std::size_t c = 1; c < NumCandidates; ++c)
        {
            const auto candidate = LcgSample();
            const float ei = ExpectedImprovement(candidate, bestValue);
            if (ei > bestEI)
            {
                bestEI = ei;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    OPTIMIZE_FOR_SPEED
        typename BayesianOptimization<NumParams, MaxObservations, NumCandidates>::Result
        BayesianOptimization<NumParams, MaxObservations, NumCandidates>::Optimize(
            BlackBoxObjective<NumParams>& objective,
            std::size_t numIterations)
    {
        really_assert(numIterations > 0 && numObservations_ + numIterations <= MaxObservations);

        const std::size_t explorationCount = std::min(std::size_t{ 5 }, numIterations / 3 + 1);

        for (std::size_t i = 0; i < explorationCount; ++i)
        {
            const auto point = LcgSample();
            const float val = objective.Evaluate(point);
            AddObservation(point, val);
        }

        for (std::size_t i = explorationCount; i < numIterations; ++i)
        {
            float bestValue = observedValues_[0];
            for (std::size_t j = 1; j < numObservations_; ++j)
                bestValue = std::min(bestValue, observedValues_[j]);

            const auto next = MaximizeAcquisition(bestValue);
            const float val = objective.Evaluate(next);
            AddObservation(next, val);
        }

        std::size_t bestIdx = 0;
        for (std::size_t j = 1; j < numObservations_; ++j)
            if (observedValues_[j] < observedValues_[bestIdx])
                bestIdx = j;

        return Result{ observedPoints_[bestIdx], observedValues_[bestIdx], numObservations_ };
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    const std::array<float, MaxObservations>&
    BayesianOptimization<NumParams, MaxObservations, NumCandidates>::GetObservedValues() const
    {
        return observedValues_;
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    std::size_t BayesianOptimization<NumParams, MaxObservations, NumCandidates>::GetNumObservations() const
    {
        return numObservations_;
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    float BayesianOptimization<NumParams, MaxObservations, NumCandidates>::SquaredExpKernel(
        const ParameterVector& a,
        const ParameterVector& b,
        const GpHyperparameters& hp)
    {
        float dist2 = 0.0f;
        for (std::size_t d = 0; d < NumParams; ++d)
        {
            const float diff = a.at(d, 0) - b.at(d, 0);
            dist2 += diff * diff;
        }
        return hp.signalVariance * hp.signalVariance *
               std::exp(-dist2 / (2.0f * hp.lengthScale * hp.lengthScale));
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    OPTIMIZE_FOR_SPEED
        typename BayesianOptimization<NumParams, MaxObservations, NumCandidates>::ParameterVector
        BayesianOptimization<NumParams, MaxObservations, NumCandidates>::LcgSample()
    {
        ParameterVector result{};
        for (std::size_t d = 0; d < NumParams; ++d)
        {
            lcgState_ = lcgState_ * 6364136223846793005ULL + 1442695040888963407ULL;
            const float t = static_cast<float>(lcgState_ >> 11) * (1.0f / static_cast<float>(1ULL << 53));
            result.at(d, 0) = bounds_[d].first + t * (bounds_[d].second - bounds_[d].first);
        }
        return result;
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    float BayesianOptimization<NumParams, MaxObservations, NumCandidates>::NormalCdf(float z)
    {
        return 0.5f * std::erfc(-z * std::numbers::sqrt2_v<float> * 0.5f);
    }

    template<std::size_t NumParams, std::size_t MaxObservations, std::size_t NumCandidates>
    float BayesianOptimization<NumParams, MaxObservations, NumCandidates>::NormalPdf(float z)
    {
        return std::exp(-0.5f * z * z) / std::sqrt(2.0f * std::numbers::pi_v<float>);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class BayesianOptimization<1, 10, 50>;
    extern template class BayesianOptimization<2, 30, 200>;
#endif
}
