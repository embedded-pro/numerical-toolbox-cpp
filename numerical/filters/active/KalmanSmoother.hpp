#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/CholeskyDecomposition.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cmath>
#include <numbers>

namespace filters
{
    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    class KalmanSmoother
    {
        static_assert(StateSize > 0, "StateSize must be positive");
        static_assert(MeasurementSize > 0, "MeasurementSize must be positive");
        static_assert(MaxSteps >= 2, "MaxSteps must be at least 2");

    public:
        using StateVector = math::Vector<float, StateSize>;
        using StateMatrix = math::SquareMatrix<float, StateSize>;
        using MeasurementMatrix = math::Matrix<float, MeasurementSize, StateSize>;
        using MeasurementVector = math::Vector<float, MeasurementSize>;
        using MeasurementCovariance = math::SquareMatrix<float, MeasurementSize>;
        using KalmanGainMatrix = math::Matrix<float, StateSize, MeasurementSize>;

        struct SmootherOutput
        {
            std::array<StateVector, MaxSteps> smoothedMeans{};
            std::array<StateMatrix, MaxSteps> smoothedCovariances{};
            // lagCrossCovariances[t] = P_{t, t-1 | T}
            // Index 0 is always zero (undefined); indices 1..numSteps-1 are valid
            std::array<StateMatrix, MaxSteps> lagCrossCovariances{};
            float logLikelihood{};
        };

        KalmanSmoother() = default;

        OPTIMIZE_FOR_SPEED SmootherOutput Smooth(
            const StateMatrix& F,
            const MeasurementMatrix& H,
            const StateMatrix& Q,
            const MeasurementCovariance& R,
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const StateVector& initialState,
            const StateMatrix& initialCovariance);

    private:
        std::array<StateVector, MaxSteps> filteredMeans_{};
        std::array<StateMatrix, MaxSteps> filteredCovariances_{};
        std::array<StateVector, MaxSteps> predictedMeans_{};
        std::array<StateMatrix, MaxSteps> predictedCovariances_{};
        std::array<KalmanGainMatrix, MaxSteps> kalmanGains_{};

        OPTIMIZE_FOR_SPEED float RunForwardPass(
            const StateMatrix& F,
            const MeasurementMatrix& H,
            const StateMatrix& Q,
            const MeasurementCovariance& R,
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const StateVector& initialState,
            const StateMatrix& initialCovariance);

        OPTIMIZE_FOR_SPEED void RunBackwardPass(
            const StateMatrix& F,
            const MeasurementMatrix& H,
            std::size_t numSteps,
            SmootherOutput& output);
    };

    // Implementation //

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED
        typename KalmanSmoother<StateSize, MeasurementSize, MaxSteps>::SmootherOutput
        KalmanSmoother<StateSize, MeasurementSize, MaxSteps>::Smooth(
            const StateMatrix& F,
            const MeasurementMatrix& H,
            const StateMatrix& Q,
            const MeasurementCovariance& R,
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const StateVector& initialState,
            const StateMatrix& initialCovariance)
    {
        really_assert(numSteps >= 2 && numSteps <= MaxSteps);

        SmootherOutput output;
        output.logLikelihood = RunForwardPass(F, H, Q, R, observations, numSteps, initialState, initialCovariance);
        RunBackwardPass(F, H, numSteps, output);
        return output;
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED float
    KalmanSmoother<StateSize, MeasurementSize, MaxSteps>::RunForwardPass(
        const StateMatrix& F,
        const MeasurementMatrix& H,
        const StateMatrix& Q,
        const MeasurementCovariance& R,
        const std::array<MeasurementVector, MaxSteps>& observations,
        std::size_t numSteps,
        const StateVector& initialState,
        const StateMatrix& initialCovariance)
    {
        predictedMeans_[0] = initialState;
        predictedCovariances_[0] = initialCovariance;

        float logLikelihood = 0.0f;
        const float logTwoPi = std::log(2.0f * std::numbers::pi_v<float>);

        for (std::size_t t = 0; t < numSteps; ++t)
        {
            const auto nu = observations[t] - H * predictedMeans_[t];
            const auto S = H * predictedCovariances_[t] * H.Transpose() + R;

            // Kalman gain: K = P H^T S^{-1}  via SolveSystem(S, H*P)^T
            const auto K = solvers::SolveSystem<float, MeasurementSize, StateSize>(
                S, H * predictedCovariances_[t])
                               .Transpose();

            filteredMeans_[t] = predictedMeans_[t] + K * nu;

            // Joseph-form covariance update for numerical PD-preservation
            const auto IminusKH = StateMatrix::Identity() - K * H;
            filteredCovariances_[t] =
                IminusKH * predictedCovariances_[t] * IminusKH.Transpose() + K * R * K.Transpose();

            kalmanGains_[t] = K;

            // Log-likelihood contribution: -0.5 * (log det S + nu^T S^{-1} nu + M*log(2pi))
            const auto L = solvers::CholeskyDecomposition(S);
            const auto S_inv_nu = solvers::SolveSystem<float, MeasurementSize, 1>(S, nu);
            float logDetS = 0.0f;
            float quadForm = 0.0f;
            for (std::size_t i = 0; i < MeasurementSize; ++i)
            {
                logDetS += std::log(L.at(i, i));
                quadForm += nu.at(i, 0) * S_inv_nu.at(i, 0);
            }
            logDetS *= 2.0f;
            logLikelihood += -0.5f * (logDetS + quadForm + static_cast<float>(MeasurementSize) * logTwoPi);

            if (t < numSteps - 1)
            {
                predictedMeans_[t + 1] = F * filteredMeans_[t];
                predictedCovariances_[t + 1] = F * filteredCovariances_[t] * F.Transpose() + Q;
            }
        }

        return logLikelihood;
    }

    template<std::size_t StateSize, std::size_t MeasurementSize, std::size_t MaxSteps>
    OPTIMIZE_FOR_SPEED void
    KalmanSmoother<StateSize, MeasurementSize, MaxSteps>::RunBackwardPass(
        const StateMatrix& F,
        const MeasurementMatrix& H,
        std::size_t numSteps,
        SmootherOutput& output)
    {
        output.smoothedMeans[numSteps - 1] = filteredMeans_[numSteps - 1];
        output.smoothedCovariances[numSteps - 1] = filteredCovariances_[numSteps - 1];

        StateMatrix G_prev{};

        for (std::size_t t = numSteps - 2;; --t)
        {
            // RTS smoother gain: G_t = P_{t|t} F^T (P_{t+1|t})^{-1}
            // Solve P_{t+1|t} * G_t^T = F * P_{t|t}  then transpose
            const auto G_t = solvers::SolveSystem<float, StateSize, StateSize>(
                predictedCovariances_[t + 1],
                F * filteredCovariances_[t])
                                 .Transpose();

            const auto deltaP = output.smoothedCovariances[t + 1] - predictedCovariances_[t + 1];
            output.smoothedMeans[t] = filteredMeans_[t] + G_t * (output.smoothedMeans[t + 1] - predictedMeans_[t + 1]);
            output.smoothedCovariances[t] = filteredCovariances_[t] + G_t * deltaP * G_t.Transpose();

            // Lag-1 cross-covariance: P_{t+1, t | T}
            if (t == numSteps - 2)
            {
                // Initialization: P_{T-1, T-2 | T} = (I - K_{T-1} H) F P_{T-2 | T-2}
                const auto IminusKH = StateMatrix::Identity() - kalmanGains_[numSteps - 1] * H;
                output.lagCrossCovariances[numSteps - 1] =
                    IminusKH * F * filteredCovariances_[numSteps - 2];
            }
            else
            {
                // Recursion: P_{t+2, t+1 | T} = P_{t+1|t+1} G_t^T + G_{t+1} (P_{t+2,t+1|T} - F P_{t+1|t+1}) G_t^T
                const auto crossDiff = output.lagCrossCovariances[t + 2] - F * filteredCovariances_[t + 1];
                output.lagCrossCovariances[t + 1] =
                    filteredCovariances_[t + 1] * G_t.Transpose() + G_prev * crossDiff * G_t.Transpose();
            }

            G_prev = G_t;

            if (t == 0)
                break;
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class KalmanSmoother<2, 1, 10>;
    extern template class KalmanSmoother<4, 2, 20>;
#endif
}
