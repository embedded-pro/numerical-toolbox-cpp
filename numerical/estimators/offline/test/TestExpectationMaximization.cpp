#include "numerical/estimators/offline/ExpectationMaximization.hpp"
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

        // Deliberately wrong initial guess — F = I, H known, Q/R inflated
        EmParameters initialGuess{
            StateMatrix::Identity(),
            MeasurementMatrix{ { 1.0f, 0.0f } },
            StateMatrix::Identity() * 0.1f,
            MeasurementCovariance::Identity() * 0.5f,
            StateVector{},
            StateMatrix::Identity()
        };

        // Hardcoded synthetic observations from true model (noisy position)
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

TEST_F(TestExpectationMaximization, log_likelihood_improves_over_iterations)
{
    // One iteration versus ten: the log-likelihood should increase meaningfully.
    const float noConverge = std::numeric_limits<float>::lowest();
    const auto result1 = em.Run(observations, T, initialGuess, 1, noConverge);
    Em em10;
    const auto result10 = em10.Run(observations, T, initialGuess, 10, noConverge);

    EXPECT_GT(result10.logLikelihood, result1.logLikelihood + 0.1f);
}

TEST_F(TestExpectationMaximization, iteration_count_is_plausible)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    EXPECT_GE(result.iterations, std::size_t{ 2 });
    EXPECT_LE(result.iterations, std::size_t{ 50 });
}

TEST_F(TestExpectationMaximization, log_likelihood_is_finite)
{
    // The Kalman log-likelihood may be positive for a tight model (small R);
    // it is not bounded above. Only finiteness is guaranteed.
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    EXPECT_TRUE(std::isfinite(result.logLikelihood));
}

TEST_F(TestExpectationMaximization, process_noise_is_symmetric)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            EXPECT_NEAR(result.parameters.Q.at(i, j), result.parameters.Q.at(j, i), 1e-5f);
}

TEST_F(TestExpectationMaximization, measurement_noise_is_symmetric)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    for (std::size_t i = 0; i < M; ++i)
        for (std::size_t j = 0; j < M; ++j)
            EXPECT_NEAR(result.parameters.R.at(i, j), result.parameters.R.at(j, i), 1e-5f);
}

TEST_F(TestExpectationMaximization, process_noise_diagonal_is_positive)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_GT(result.parameters.Q.at(i, i), 0.0f);
}

TEST_F(TestExpectationMaximization, measurement_noise_diagonal_is_positive)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    for (std::size_t i = 0; i < M; ++i)
        EXPECT_GT(result.parameters.R.at(i, i), 0.0f);
}

TEST_F(TestExpectationMaximization, initial_state_is_plausible)
{
    const auto result = em.Run(observations, T, initialGuess, 50, 1e-3f);

    EXPECT_GT(result.parameters.initialState.at(0, 0), -2.0f);
    EXPECT_LT(result.parameters.initialState.at(0, 0), 2.0f);
}

TEST_F(TestExpectationMaximization, recovered_parameters_produce_finite_predictions)
{
    // With any converged parameters, running the smoother again must not produce NaN/Inf.
    const float noConverge = std::numeric_limits<float>::lowest();
    const auto result = em.Run(observations, T, initialGuess, 20, noConverge);

    Em em2;
    const auto result2 = em2.Run(observations, T, result.parameters, 1, noConverge);

    EXPECT_TRUE(std::isfinite(result2.logLikelihood));
}

TEST_F(TestExpectationMaximization, log_likelihood_increases_monotonically)
{
    // Run EM one step at a time, feeding output as next input; verify non-decreasing log-likelihood
    const float neverConverge = std::numeric_limits<float>::lowest();
    EmParameters params = initialGuess;
    float prevLl = neverConverge;

    for (std::size_t step = 0; step < 5; ++step)
    {
        const auto result = em.Run(observations, T, params, 1, neverConverge);
        if (prevLl != neverConverge)
            EXPECT_GE(result.logLikelihood, prevLl - 1e-3f);
        prevLl = result.logLikelihood;
        params = result.parameters;
    }
}
