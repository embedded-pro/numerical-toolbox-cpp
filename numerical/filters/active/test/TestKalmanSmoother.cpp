#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/filters/active/KalmanSmoother.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <random>

namespace
{
    class TestKalmanSmoother : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 2;
        static constexpr std::size_t M = 1;
        static constexpr std::size_t T = 10;

        using Smoother = filters::KalmanSmoother<N, M, T>;
        using StateVector = Smoother::StateVector;
        using StateMatrix = Smoother::StateMatrix;
        using MeasurementMatrix = Smoother::MeasurementMatrix;
        using MeasurementVector = Smoother::MeasurementVector;
        using MeasurementCovariance = Smoother::MeasurementCovariance;
        using Cm2 = math::ConsistencyMetrics<float, N>;

        Smoother smoother;

        StateMatrix F{ { 1.0f, 0.1f }, { 0.0f, 1.0f } };
        MeasurementMatrix H{ { 1.0f, 0.0f } };
        StateMatrix Q{ { 0.01f, 0.0f }, { 0.0f, 0.01f } };
        MeasurementCovariance R{ { 0.5f } };
        StateVector x0{ { 0.0f }, { 0.0f } };
        StateMatrix P0{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

        std::array<MeasurementVector, T> observations{ { MeasurementVector{ { 0.07f } },
            MeasurementVector{ { 0.15f } },
            MeasurementVector{ { 0.22f } },
            MeasurementVector{ { 0.28f } },
            MeasurementVector{ { 0.36f } },
            MeasurementVector{ { 0.43f } },
            MeasurementVector{ { 0.52f } },
            MeasurementVector{ { 0.59f } },
            MeasurementVector{ { 0.64f } },
            MeasurementVector{ { 0.73f } } } };

        Smoother::SmootherOutput RunSmoother()
        {
            return smoother.Smooth({F, H, Q, R}, observations, T, x0, P0);
        }
    };

    class TestKalmanSmootherScalar : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 1;
        static constexpr std::size_t M = 1;
        static constexpr std::size_t T = 3;

        using Smoother = filters::KalmanSmoother<N, M, T>;
        using StateVector = Smoother::StateVector;
        using StateMatrix = Smoother::StateMatrix;
        using MeasurementMatrix = Smoother::MeasurementMatrix;
        using MeasurementVector = Smoother::MeasurementVector;
        using MeasurementCovariance = Smoother::MeasurementCovariance;

        Smoother smoother;

        StateMatrix F{ { 1.0f } };
        MeasurementMatrix H{ { 1.0f } };
        StateMatrix Q{ { 0.1f } };
        MeasurementCovariance R{ { 1.0f } };
        StateVector x0{ { 0.0f } };
        StateMatrix P0{ { 1.0f } };

        std::array<MeasurementVector, T> obs{ { MeasurementVector{ { 0.5f } },
            MeasurementVector{ { 0.3f } },
            MeasurementVector{ { 0.8f } } } };

        Smoother::SmootherOutput RunSmoother()
        {
            return smoother.Smooth({F, H, Q, R}, obs, T, x0, P0);
        }
    };
}

TEST_F(TestKalmanSmootherScalar, smoothed_means_match_hand_computed_reference)
{
    const auto out = RunSmoother();

    EXPECT_NEAR(out.smoothedMeans[2].at(0, 0), 0.43983f, 1e-3f);
    EXPECT_NEAR(out.smoothedMeans[1].at(0, 0), 0.40381f, 1e-3f);
    EXPECT_NEAR(out.smoothedMeans[0].at(0, 0), 0.37818f, 1e-3f);
}

TEST_F(TestKalmanSmootherScalar, smoothed_covariances_match_hand_computed_reference)
{
    const auto out = RunSmoother();

    EXPECT_NEAR(out.smoothedCovariances[2].at(0, 0), 0.32203f, 1e-3f);
    EXPECT_NEAR(out.smoothedCovariances[1].at(0, 0), 0.27962f, 1e-3f);
    EXPECT_NEAR(out.smoothedCovariances[0].at(0, 0), 0.27751f, 1e-3f);
}

TEST_F(TestKalmanSmootherScalar, log_likelihood_matches_hand_computed_reference)
{
    const auto out = RunSmoother();

    EXPECT_NEAR(out.logLikelihood, -3.6916f, 5e-3f);
}

TEST_F(TestKalmanSmootherScalar, lag_cross_covariance_index_zero_is_always_zero)
{
    const auto out = RunSmoother();

    EXPECT_FLOAT_EQ(out.lagCrossCovariances[0].at(0, 0), 0.0f);
}

TEST_F(TestKalmanSmootherScalar, lag_cross_covariance_last_step_is_nonzero)
{
    const auto out = RunSmoother();

    EXPECT_NE(out.lagCrossCovariances[T - 1].at(0, 0), 0.0f);
}

TEST_F(TestKalmanSmootherScalar, determinism_same_input_gives_identical_output)
{
    const auto out1 = RunSmoother();
    const auto out2 = RunSmoother();

    for (std::size_t t = 0; t < T; ++t)
    {
        EXPECT_FLOAT_EQ(out1.smoothedMeans[t].at(0, 0), out2.smoothedMeans[t].at(0, 0));
        EXPECT_FLOAT_EQ(out1.smoothedCovariances[t].at(0, 0), out2.smoothedCovariances[t].at(0, 0));
    }
    EXPECT_FLOAT_EQ(out1.logLikelihood, out2.logLikelihood);
}

TEST_F(TestKalmanSmoother, smoothed_covariances_are_no_larger_than_filtered)
{
    filters::KalmanFilter<float, N, M> kf(x0, P0);
    kf.SetStateTransition(F);
    kf.SetMeasurementMatrix(H);
    kf.SetProcessNoise(Q);
    kf.SetMeasurementNoise(R);

    std::array<float, T> filteredVar{};
    kf.Update(observations[0]);
    filteredVar[0] = kf.GetCovariance().at(0, 0);
    for (std::size_t t = 1; t < T; ++t)
    {
        kf.Predict();
        kf.Update(observations[t]);
        filteredVar[t] = kf.GetCovariance().at(0, 0);
    }

    const auto output = RunSmoother();

    for (std::size_t t = 0; t < T; ++t)
        EXPECT_LE(output.smoothedCovariances[t].at(0, 0), filteredVar[t] + 1e-5f);
}

TEST_F(TestKalmanSmoother, smoothed_covariance_is_symmetric_positive_definite)
{
    const auto output = RunSmoother();

    for (std::size_t t = 0; t < T; ++t)
    {
        const auto& P = output.smoothedCovariances[t];
        EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
        EXPECT_GT(P.at(0, 0), 0.0f);
        EXPECT_GT(P.at(1, 1), 0.0f);
        const float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
        EXPECT_GT(det, 0.0f);
    }
}

TEST_F(TestKalmanSmoother, smoothed_rmse_is_less_than_filtered_rmse)
{
    constexpr float trueVel = 0.1f;
    std::mt19937 rng{ 7u };
    std::normal_distribution<float> measNoise{ 0.0f, std::sqrt(0.5f) };

    std::array<StateVector, T> trueStates{};
    std::array<MeasurementVector, T> syntheticObs{};
    trueStates[0] = StateVector{ { 0.0f }, { trueVel } };
    syntheticObs[0] = MeasurementVector{ { trueStates[0].at(0, 0) + measNoise(rng) } };
    for (std::size_t t = 1; t < T; ++t)
    {
        trueStates[t].at(0, 0) = F.at(0, 0) * trueStates[t - 1].at(0, 0) + F.at(0, 1) * trueStates[t - 1].at(1, 0);
        trueStates[t].at(1, 0) = trueVel;
        syntheticObs[t] = MeasurementVector{ { trueStates[t].at(0, 0) + measNoise(rng) } };
    }

    filters::KalmanFilter<float, N, M> kf(x0, P0);
    kf.SetStateTransition(F);
    kf.SetMeasurementMatrix(H);
    kf.SetProcessNoise(Q);
    kf.SetMeasurementNoise(R);

    std::array<float, T> filteredPos{};
    kf.Update(syntheticObs[0]);
    filteredPos[0] = kf.GetState().at(0, 0);
    for (std::size_t t = 1; t < T; ++t)
    {
        kf.Predict();
        kf.Update(syntheticObs[t]);
        filteredPos[t] = kf.GetState().at(0, 0);
    }

    Smoother synthSmoother;
    const auto output = synthSmoother.Smooth({F, H, Q, R}, syntheticObs, T, x0, P0);

    float filteredMse = 0.0f;
    float smoothedMse = 0.0f;
    for (std::size_t t = 0; t < T; ++t)
    {
        const float truePos = trueStates[t].at(0, 0);
        const float ef = filteredPos[t] - truePos;
        const float es = output.smoothedMeans[t].at(0, 0) - truePos;
        filteredMse += ef * ef;
        smoothedMse += es * es;
    }

    EXPECT_LE(smoothedMse, filteredMse);
}

TEST_F(TestKalmanSmoother, log_likelihood_is_finite_and_negative)
{
    const auto output = RunSmoother();

    EXPECT_TRUE(std::isfinite(output.logLikelihood));
    EXPECT_LT(output.logLikelihood, 0.0f);
}

TEST_F(TestKalmanSmoother, smooth_with_lti_matches_smooth_with_matrices)
{
    auto plant = math::LinearTimeInvariant<float, N, 1, M>{ F, {}, H, {} };

    const auto outputMatrices = RunSmoother();
    const auto outputLti = smoother.Smooth(plant, Q, R, observations, T, x0, P0);

    EXPECT_NEAR(outputMatrices.smoothedMeans[0].at(0, 0), outputLti.smoothedMeans[0].at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(outputMatrices.smoothedMeans[0].at(1, 0), outputLti.smoothedMeans[0].at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(outputMatrices.logLikelihood, outputLti.logLikelihood, math::Tolerance<float>());
}

TEST_F(TestKalmanSmoother, smoothed_covariance_at_final_step_equals_filtered_covariance)
{
    filters::KalmanFilter<float, N, M> kf(x0, P0);
    kf.SetStateTransition(F);
    kf.SetMeasurementMatrix(H);
    kf.SetProcessNoise(Q);
    kf.SetMeasurementNoise(R);

    kf.Update(observations[0]);
    for (std::size_t t = 1; t < T; ++t)
    {
        kf.Predict();
        kf.Update(observations[t]);
    }
    const float filteredFinalVar = kf.GetCovariance().at(0, 0);

    const auto output = RunSmoother();

    EXPECT_NEAR(output.smoothedCovariances[T - 1].at(0, 0), filteredFinalVar, math::Tolerance<float>());
}

TEST_F(TestKalmanSmoother, minimum_steps_boundary_produces_finite_output)
{
    constexpr std::size_t minSteps = 2;
    const auto output = smoother.Smooth({F, H, Q, R}, observations, minSteps, x0, P0);

    EXPECT_TRUE(std::isfinite(output.logLikelihood));
    for (std::size_t t = 0; t < minSteps; ++t)
        for (std::size_t i = 0; i < N; ++i)
            EXPECT_TRUE(std::isfinite(output.smoothedMeans[t].at(i, 0)));
}

TEST_F(TestKalmanSmoother, near_zero_process_noise_produces_no_nan)
{
    StateMatrix Qsmall{ { 1e-6f, 0.0f }, { 0.0f, 1e-6f } };
    const auto output = smoother.Smooth({F, H, Qsmall, R}, observations, T, x0, P0);

    EXPECT_TRUE(std::isfinite(output.logLikelihood));
    for (std::size_t t = 0; t < T; ++t)
    {
        for (std::size_t i = 0; i < N; ++i)
            EXPECT_TRUE(std::isfinite(output.smoothedMeans[t].at(i, 0)));
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                EXPECT_TRUE(std::isfinite(output.smoothedCovariances[t].at(i, j)));
    }
}

TEST_F(TestKalmanSmoother, nees_within_chi_squared_band_on_simulated_system)
{
    constexpr std::size_t NumTrials = 30;
    constexpr float trueVel = 0.1f;
    constexpr float dt = 0.1f;

    std::mt19937 rng{ 42u };
    std::normal_distribution<float> procNoise{ 0.0f, 0.1f };
    std::normal_distribution<float> measNoise{ 0.0f, std::sqrt(0.5f) };

    float neesSum = 0.0f;
    std::size_t neesCount = 0;

    for (std::size_t trial = 0; trial < NumTrials; ++trial)
    {
        std::array<MeasurementVector, T> trialObs{};
        std::array<StateVector, T> trialTrue{};
        trialTrue[0] = StateVector{ { 0.0f }, { trueVel } };
        for (std::size_t t = 1; t < T; ++t)
        {
            trialTrue[t].at(0, 0) = trialTrue[t - 1].at(0, 0) + dt * trialTrue[t - 1].at(1, 0) + procNoise(rng);
            trialTrue[t].at(1, 0) = trueVel + procNoise(rng);
        }
        for (std::size_t t = 0; t < T; ++t)
            trialObs[t] = MeasurementVector{ { trialTrue[t].at(0, 0) + measNoise(rng) } };

        Smoother trialSmoother;
        const auto out = trialSmoother.Smooth({F, H, Q, R}, trialObs, T, x0, P0);

        for (std::size_t t = 0; t < T; ++t)
        {
            const StateVector error{ { out.smoothedMeans[t].at(0, 0) - trialTrue[t].at(0, 0) },
                { out.smoothedMeans[t].at(1, 0) - trialTrue[t].at(1, 0) } };
            const auto nees = Cm2::Nees(error, out.smoothedCovariances[t]);
            if (nees.has_value())
            {
                neesSum += nees.value();
                ++neesCount;
            }
        }
    }

    ASSERT_GT(neesCount, 0u);
    const float neesAvg = neesSum / static_cast<float>(neesCount);
    EXPECT_TRUE(Cm2::IsConsistent(neesAvg));
}
