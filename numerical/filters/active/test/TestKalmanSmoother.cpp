#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/filters/active/KalmanSmoother.hpp"
#include <cmath>
#include <gtest/gtest.h>

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

        Smoother smoother;

        // Constant-velocity model: x = [position, velocity], z = position, dt = 0.1 s
        StateMatrix F{ { 1.0f, 0.1f }, { 0.0f, 1.0f } };
        MeasurementMatrix H{ { 1.0f, 0.0f } };
        StateMatrix Q{ { 0.01f, 0.0f }, { 0.0f, 0.01f } };
        MeasurementCovariance R{ { 0.5f } };
        StateVector x0{ { 0.0f }, { 0.0f } };
        StateMatrix P0{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

        // Noisy position observations from a constant-velocity trajectory (v = 0.5 m/s)
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
    };
}

TEST_F(TestKalmanSmoother, log_likelihood_is_finite_and_negative)
{
    const auto output = smoother.Smooth(F, H, Q, R, observations, T, x0, P0);

    EXPECT_TRUE(std::isfinite(output.logLikelihood));
    EXPECT_LT(output.logLikelihood, 0.0f);
}

TEST_F(TestKalmanSmoother, lag_cross_covariance_index_zero_is_zero)
{
    const auto output = smoother.Smooth(F, H, Q, R, observations, T, x0, P0);

    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            EXPECT_FLOAT_EQ(output.lagCrossCovariances[0].at(i, j), 0.0f);
}

TEST_F(TestKalmanSmoother, lag_cross_covariance_last_index_is_nonzero)
{
    const auto output = smoother.Smooth(F, H, Q, R, observations, T, x0, P0);

    float norm = 0.0f;
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            norm += output.lagCrossCovariances[T - 1].at(i, j) * output.lagCrossCovariances[T - 1].at(i, j);

    EXPECT_GT(norm, 0.0f);
}

TEST_F(TestKalmanSmoother, smoothed_covariances_are_no_larger_than_filtered)
{
    // Run a KF matching the smoother's forward pass:
    // t=0: use P0 directly as prediction cov (no prior Predict), then Update
    // t=1..T-1: Predict then Update
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

    const auto output = smoother.Smooth(F, H, Q, R, observations, T, x0, P0);

    for (std::size_t t = 0; t < T; ++t)
        EXPECT_LE(output.smoothedCovariances[t].at(0, 0), filteredVar[t] + 1e-5f);
}

TEST_F(TestKalmanSmoother, smoothed_covariance_at_first_step_less_than_prior)
{
    // After observing T steps backward, uncertainty at t=0 must be less than prior P0.
    // P_{0|T} <= P_{0|0} <= P0 is guaranteed by the Kalman update and RTS smoother.
    const auto output = smoother.Smooth(F, H, Q, R, observations, T, x0, P0);

    EXPECT_LT(output.smoothedCovariances[0].at(0, 0), P0.at(0, 0));
    EXPECT_LT(output.smoothedCovariances[0].at(1, 1), P0.at(1, 1));
}
