#include "numerical/estimators/offline/ExpectationMaximization.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

namespace
{
    class TestExpectationMaximization : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 2;
        static constexpr std::size_t M = 1;
        static constexpr std::size_t T = 10;

        using Em = estimators::ExpectationMaximization<N, M, T>;
        using EmParameters = Em::EmParameters;
        using EmResult = Em::EmResult;
        using StateMatrix = Em::StateMatrix;
        using StateVector = Em::StateVector;
        using MeasurementMatrix = Em::MeasurementMatrix;
        using MeasurementVector = Em::MeasurementVector;
        using MeasurementCovariance = Em::MeasurementCovariance;

        Em em;

        EmParameters MakeInitialGuess()
        {
            return EmParameters{
                StateMatrix::Identity(),
                MeasurementMatrix{ { 1.0f, 0.0f } },
                StateMatrix::Identity() * 0.1f,
                MeasurementCovariance::Identity() * 0.5f,
                StateVector{},
                StateMatrix::Identity()
            };
        }

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

    class TestExpectationMaximizationRecovery : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N4 = 4;
        static constexpr std::size_t M2 = 2;
        static constexpr std::size_t T20 = 20;

        using Em4 = estimators::ExpectationMaximization<N4, M2, T20>;
        using EmParameters4 = Em4::EmParameters;
        using StateMatrix4 = Em4::StateMatrix;
        using StateVector4 = Em4::StateVector;
        using MeasurementMatrix4 = Em4::MeasurementMatrix;
        using MeasurementVector4 = Em4::MeasurementVector;
        using MeasurementCovariance4 = Em4::MeasurementCovariance;

        Em4 em;

        EmParameters4 MakeInitialGuess()
        {
            return EmParameters4{
                StateMatrix4::Identity() * 0.8f,
                MeasurementMatrix4{ { 0.5f, 0.5f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.5f, 0.5f } },
                StateMatrix4::Identity() * 0.1f,
                MeasurementCovariance4::Identity() * 0.3f,
                StateVector4{},
                StateMatrix4::Identity()
            };
        }

        std::array<MeasurementVector4, T20> MakeObservations()
        {
            std::array<MeasurementVector4, T20> obs{};
            for (std::size_t t = 0; t < T20; ++t)
            {
                const float v = static_cast<float>(t) * 0.05f;
                obs[t] = MeasurementVector4{ { v }, { v * 0.5f } };
            }
            return obs;
        }
    };
}

TEST_F(TestExpectationMaximization, log_likelihood_improves_over_iterations)
{
    const float noConverge = std::numeric_limits<float>::lowest();
    const auto result1 = em.Run(observations, T, MakeInitialGuess(), 1, noConverge);
    Em em10;
    const auto result10 = em10.Run(observations, T, MakeInitialGuess(), 10, noConverge);

    EXPECT_GT(result10.logLikelihood, result1.logLikelihood + 0.1f);
}

TEST_F(TestExpectationMaximization, converged_flag_reflects_tolerance_outcome)
{
    EXPECT_FALSE(em.Run(observations, T, MakeInitialGuess(), 1, 1e-6f).converged);
    Em em2;
    EXPECT_TRUE(em2.Run(observations, T, MakeInitialGuess(), 50, 1000.0f).converged);
}

TEST_F(TestExpectationMaximization, deterministic_repeated_runs_agree)
{
    const float noConverge = std::numeric_limits<float>::lowest();
    const auto r1 = em.Run(observations, T, MakeInitialGuess(), 5, noConverge);
    Em em2;
    const auto r2 = em2.Run(observations, T, MakeInitialGuess(), 5, noConverge);

    EXPECT_FLOAT_EQ(r1.logLikelihood, r2.logLikelihood);
    EXPECT_FLOAT_EQ(r1.parameters.H.at(0, 0), r2.parameters.H.at(0, 0));
    EXPECT_FLOAT_EQ(r1.parameters.H.at(0, 1), r2.parameters.H.at(0, 1));
}

TEST_F(TestExpectationMaximization, log_likelihood_increases_monotonically)
{
    const float neverConverge = std::numeric_limits<float>::lowest();
    EmParameters params = MakeInitialGuess();
    float prevLl = neverConverge;

    for (std::size_t step = 0; step < 8; ++step)
    {
        Em emStep;
        const auto result = emStep.Run(observations, T, params, 1, neverConverge);
        if (prevLl != neverConverge)
            EXPECT_GE(result.logLikelihood, prevLl - math::Tolerance<float>());
        prevLl = result.logLikelihood;
        params = result.parameters;
    }
}

TEST_F(TestExpectationMaximization, noise_covariances_are_symmetric_positive_definite)
{
    const auto result = em.Run(observations, T, MakeInitialGuess(), 50, 1e-3f);

    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_GT(result.parameters.Q.at(i, i), 0.0f);
        for (std::size_t j = i + 1; j < N; ++j)
            EXPECT_NEAR(result.parameters.Q.at(i, j), result.parameters.Q.at(j, i), 1e-5f);
    }
    EXPECT_GT(result.parameters.R.at(0, 0), 0.0f);
}

TEST_F(TestExpectationMaximizationRecovery, observation_matrix_direction_recovered_from_structured_data)
{
    const auto obs = MakeObservations();
    EmParameters4 guess = MakeInitialGuess();
    const auto result = em.Run(obs, T20, guess, 80, 1e-4f);

    const float h00 = result.parameters.H.at(0, 0);
    const float h01 = result.parameters.H.at(0, 1);
    const float h10 = result.parameters.H.at(1, 0);
    const float h11 = result.parameters.H.at(1, 1);

    EXPECT_TRUE(std::isfinite(h00));
    EXPECT_TRUE(std::isfinite(h01));
    EXPECT_TRUE(std::isfinite(h10));
    EXPECT_TRUE(std::isfinite(h11));

    const float hNorm0 = std::sqrt(h00 * h00 + h01 * h01 + result.parameters.H.at(0, 2) * result.parameters.H.at(0, 2) + result.parameters.H.at(0, 3) * result.parameters.H.at(0, 3));
    const float hNorm1 = std::sqrt(h10 * h10 + h11 * h11 + result.parameters.H.at(1, 2) * result.parameters.H.at(1, 2) + result.parameters.H.at(1, 3) * result.parameters.H.at(1, 3));
    EXPECT_GT(hNorm0, 0.05f);
    EXPECT_GT(hNorm1, 0.05f);

    EXPECT_GT(result.logLikelihood, std::numeric_limits<float>::lowest() + 1.0f);
    EXPECT_TRUE(std::isfinite(result.logLikelihood));

    for (std::size_t i = 0; i < N4; ++i)
    {
        EXPECT_GT(result.parameters.Q.at(i, i), 0.0f);
        EXPECT_TRUE(std::isfinite(result.parameters.Q.at(i, i)));
    }
    for (std::size_t i = 0; i < M2; ++i)
        EXPECT_GT(result.parameters.R.at(i, i), 0.0f);
}

TEST_F(TestExpectationMaximization, minimal_steps_returns_finite_symmetric_covariances)
{
    std::array<MeasurementVector, T> minObs{};
    minObs[0] = MeasurementVector{ { 0.1f } };
    minObs[1] = MeasurementVector{ { 0.2f } };

    const auto result = em.Run(minObs, 2, MakeInitialGuess(), 5, 1e-6f);

    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_TRUE(std::isfinite(result.parameters.Q.at(i, i)));
        EXPECT_GT(result.parameters.Q.at(i, i), 0.0f);
        for (std::size_t j = i + 1; j < N; ++j)
            EXPECT_NEAR(result.parameters.Q.at(i, j), result.parameters.Q.at(j, i), 1e-4f);
    }
    EXPECT_TRUE(std::isfinite(result.parameters.R.at(0, 0)));
    EXPECT_GT(result.parameters.R.at(0, 0), 0.0f);
    EXPECT_TRUE(std::isfinite(result.logLikelihood));
}
