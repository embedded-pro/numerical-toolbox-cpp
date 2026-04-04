#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/filters/active/KalmanSmoother.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace estimators
{
    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    class ExpectationMaximization
    {
        static_assert(StateSize > 0, "StateSize must be positive");
        static_assert(MeasurementSize > 0, "MeasurementSize must be positive");
        static_assert(MaxSteps >= 2, "MaxSteps must be at least 2 for EM");

    public:
        using StateVector = math::Vector<float, StateSize>;
        using StateMatrix = math::SquareMatrix<float, StateSize>;
        using MeasurementMatrix = math::Matrix<float, MeasurementSize, StateSize>;
        using MeasurementVector = math::Vector<float, MeasurementSize>;
        using MeasurementCovariance = math::SquareMatrix<float, MeasurementSize>;

        struct EmParameters
        {
            StateMatrix F;
            MeasurementMatrix H;
            StateMatrix Q;
            MeasurementCovariance R;
            StateVector initialState;
            StateMatrix initialCovariance;
        };

        struct EmResult
        {
            EmParameters parameters;
            std::size_t iterations;
            float logLikelihood;
            bool converged;
        };

        ExpectationMaximization() = default;

        OPTIMIZE_FOR_SPEED EmResult Run(
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const EmParameters& initialParameters,
            std::size_t maxIterations,
            float convergenceTolerance);

    private:
        using SmootherType = filters::KalmanSmoother<StateSize, MeasurementSize, MaxSteps>;
        using SmootherOutput = typename SmootherType::SmootherOutput;

        SmootherType smoother_;

        OPTIMIZE_FOR_SPEED EmParameters MStep(
            const SmootherOutput& smootherOutput,
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps);

        OPTIMIZE_FOR_SPEED std::pair<StateMatrix, StateMatrix> ComputeTransitionUpdate(
            const StateMatrix& transitionCrossCov,
            const StateMatrix& laggedStateCov,
            const StateMatrix& currentStateCov,
            std::size_t numSteps) const;

        OPTIMIZE_FOR_SPEED std::pair<MeasurementMatrix, MeasurementCovariance> ComputeObservationUpdate(
            const math::Matrix<float, MeasurementSize, StateSize>& obsStateCrossCov,
            const StateMatrix& fullStateCov,
            const MeasurementCovariance& obsOuterSum,
            std::size_t numSteps) const;

        template<std::size_t N>
        static math::SquareMatrix<float, N> Symmetrize(const math::SquareMatrix<float, N>& M);
    };

    // Implementation //

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED
        typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::EmResult
        ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::Run(
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const EmParameters& initialParameters,
            std::size_t maxIterations,
            float convergenceTolerance)
    {
        really_assert(numSteps >= 2 && numSteps <= MaxSteps);
        really_assert(maxIterations > 0);

        EmParameters currentParams = initialParameters;
        float prevLogLikelihood = std::numeric_limits<float>::lowest();
        float logLikelihood = std::numeric_limits<float>::lowest();
        std::size_t iter = 0;
        bool converged = false;

        while (iter < maxIterations)
        {
            const auto smootherOutput = smoother_.Smooth(
                currentParams.F, currentParams.H,
                currentParams.Q, currentParams.R,
                observations, numSteps,
                currentParams.initialState, currentParams.initialCovariance);

            logLikelihood = smootherOutput.logLikelihood;
            ++iter;

            if (std::abs(logLikelihood - prevLogLikelihood) < convergenceTolerance)
            {
                converged = true;
                break;
            }
            prevLogLikelihood = logLikelihood;

            currentParams = MStep(smootherOutput, observations, numSteps);
        }

        return EmResult{ currentParams, iter, logLikelihood, converged };
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED
        typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::EmParameters
        ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::MStep(
            const SmootherOutput& smootherOutput,
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps)
    {
        StateMatrix transitionCrossCov{};
        StateMatrix laggedStateCov{};
        StateMatrix currentStateCov{};
        StateMatrix fullStateCov{};
        math::Matrix<float, MeasurementSize, StateSize> obsStateCrossCov{};
        MeasurementCovariance obsOuterSum{};

        for (std::size_t t = 0; t < numSteps; ++t)
        {
            const auto& xS = smootherOutput.smoothedMeans[t];
            const auto& PS = smootherOutput.smoothedCovariances[t];
            const auto xxT = xS * xS.Transpose();
            const auto PxxT = PS + xxT;

            if (t < numSteps - 1)
                laggedStateCov += PxxT;

            if (t >= 1)
            {
                currentStateCov += PxxT;
                transitionCrossCov += smootherOutput.lagCrossCovariances[t] + xS * smootherOutput.smoothedMeans[t - 1].Transpose();
            }

            fullStateCov += PxxT;
            obsStateCrossCov += observations[t] * xS.Transpose();
            obsOuterSum += observations[t] * observations[t].Transpose();
        }

        const auto [F_new, Q_new] = ComputeTransitionUpdate(transitionCrossCov, laggedStateCov, currentStateCov, numSteps);
        const auto [H_new, R_new] = ComputeObservationUpdate(obsStateCrossCov, fullStateCov, obsOuterSum, numSteps);

        return EmParameters{
            F_new,
            H_new,
            Q_new,
            R_new,
            smootherOutput.smoothedMeans[0],
            smootherOutput.smoothedCovariances[0]
        };
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED
        std::pair<typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::StateMatrix,
            typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::StateMatrix>
        ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::ComputeTransitionUpdate(
            const StateMatrix& transitionCrossCov,
            const StateMatrix& laggedStateCov,
            const StateMatrix& currentStateCov,
            std::size_t numSteps) const
    {
        const auto F_new = solvers::SolveSystem<float, StateSize, StateSize>(
            laggedStateCov, transitionCrossCov.Transpose())
                               .Transpose();
        const float invTm1 = 1.0f / static_cast<float>(numSteps - 1);
        auto Q_new = Symmetrize<StateSize>((currentStateCov - F_new * transitionCrossCov.Transpose()) * invTm1);
        Q_new += StateMatrix::Identity() * 1e-6f;
        return { F_new, Q_new };
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED
        std::pair<typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::MeasurementMatrix,
            typename ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::MeasurementCovariance>
        ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::ComputeObservationUpdate(
            const math::Matrix<float, MeasurementSize, StateSize>& obsStateCrossCov,
            const StateMatrix& fullStateCov,
            const MeasurementCovariance& obsOuterSum,
            std::size_t numSteps) const
    {
        const auto H_new = solvers::SolveSystem<float, StateSize, MeasurementSize>(
            fullStateCov, obsStateCrossCov.Transpose())
                               .Transpose();
        const float invT = 1.0f / static_cast<float>(numSteps);
        auto R_new = Symmetrize<MeasurementSize>((obsOuterSum - H_new * obsStateCrossCov.Transpose()) * invT);
        R_new += MeasurementCovariance::Identity() * 1e-6f;
        return { H_new, R_new };
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    template<std::size_t N>
    math::SquareMatrix<float, N>
    ExpectationMaximization<StateSize, MeasurementSize, MaxSteps>::Symmetrize(const math::SquareMatrix<float, N>& M)
    {
        return (M + M.Transpose()) * 0.5f;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ExpectationMaximization<2, 1, 10>;
    extern template class ExpectationMaximization<4, 2, 20>;
#endif
}
