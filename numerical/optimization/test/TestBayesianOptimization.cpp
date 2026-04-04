#include "numerical/optimization/BayesianOptimization.hpp"
#include "numerical/optimization/BlackBoxObjective.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestBayesianOptimization1D : public ::testing::Test
    {
    protected:
        static constexpr std::size_t NumParams = 1;
        static constexpr std::size_t MaxObs = 10;
        static constexpr std::size_t NumCandidates = 50;

        using Bo = optimization::BayesianOptimization<NumParams, MaxObs, NumCandidates>;
        using ParamVec = Bo::ParameterVector;

        optimization::GpHyperparameters hp{ 0.5f, 1.0f, 0.01f };
        Bo::BoundsArray bounds{ std::make_pair(-3.0f, 3.0f) };
        Bo bo{ bounds, hp, 99991ULL };
    };

    class TestBayesianOptimization2D : public ::testing::Test
    {
    protected:
        static constexpr std::size_t NumParams = 2;
        static constexpr std::size_t MaxObs = 20;
        static constexpr std::size_t NumCandidates = 100;

        using Bo = optimization::BayesianOptimization<NumParams, MaxObs, NumCandidates>;
        using ParamVec = Bo::ParameterVector;

        optimization::GpHyperparameters hp{ 0.8f, 1.0f, 0.01f };
        Bo::BoundsArray bounds{
            std::make_pair(-2.0f, 2.0f),
            std::make_pair(-2.0f, 2.0f)
        };
        Bo bo{ bounds, hp, 12345ULL };
    };

    class QuadraticObjective : public optimization::BlackBoxObjective<1>
    {
    public:
        float Evaluate(const math::Vector<float, 1>& params) override
        {
            const float x = params.at(0, 0);
            return x * x;
        }
    };

    class BowlObjective : public optimization::BlackBoxObjective<2>
    {
    public:
        float Evaluate(const math::Vector<float, 2>& params) override
        {
            const float dx = params.at(0, 0) - 0.4f;
            const float dy = params.at(1, 0) + 0.3f;
            return dx * dx + dy * dy;
        }
    };
}

TEST_F(TestBayesianOptimization1D, gp_predict_returns_finite_mean_and_nonneg_std_after_observations)
{
    bo.AddObservation(ParamVec{ { -1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);

    const auto [mu, sigma] = bo.GpPredict(ParamVec{ { 0.5f } });

    EXPECT_TRUE(std::isfinite(mu));
    EXPECT_GE(sigma, 0.0f);
}

TEST_F(TestBayesianOptimization1D, expected_improvement_is_nonnegative)
{
    bo.AddObservation(ParamVec{ { -1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);

    const float bestValue = 0.0f;
    const float ei = bo.ExpectedImprovement(ParamVec{ { 2.0f } }, bestValue);

    EXPECT_GE(ei, 0.0f);
}

TEST_F(TestBayesianOptimization1D, expected_improvement_is_zero_before_observations)
{
    const float ei = bo.ExpectedImprovement(ParamVec{ { 0.5f } }, 0.0f);

    EXPECT_FLOAT_EQ(ei, 0.0f);
}

TEST_F(TestBayesianOptimization1D, gp_mean_approximates_observed_value_at_observed_point)
{
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { -1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { 0.5f } }, 0.25f);

    const auto [mu, sigma] = bo.GpPredict(ParamVec{ { 1.0f } });

    EXPECT_NEAR(mu, 1.0f, 0.2f);
}

TEST_F(TestBayesianOptimization1D, optimize_1d_quadratic_finds_point_near_zero)
{
    QuadraticObjective objective;

    const auto result = bo.Optimize(objective, 8);

    EXPECT_LT(std::abs(result.bestPoint.at(0, 0)), 0.5f);
}

TEST_F(TestBayesianOptimization1D, maximize_acquisition_returns_point_within_bounds)
{
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.1f);
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);

    const auto candidate = bo.MaximizeAcquisition(0.1f);

    EXPECT_GE(candidate.at(0, 0), -3.0f);
    EXPECT_LE(candidate.at(0, 0), 3.0f);
}

TEST_F(TestBayesianOptimization2D, optimize_2d_bowl_finds_point_near_known_min)
{
    BowlObjective objective;

    const auto result = bo.Optimize(objective, 18);

    const float dx = result.bestPoint.at(0, 0) - 0.4f;
    const float dy = result.bestPoint.at(1, 0) + 0.3f;
    EXPECT_LT(std::sqrt(dx * dx + dy * dy), 0.6f);
}

TEST_F(TestBayesianOptimization2D, all_sampled_candidates_stay_within_bounds)
{
    bo.AddObservation(ParamVec{ { 0.0f }, { 0.0f } }, 0.25f);
    bo.AddObservation(ParamVec{ { 1.0f }, { -1.0f } }, 2.1f);

    for (std::size_t i = 0; i < 5; ++i)
    {
        const auto candidate = bo.MaximizeAcquisition(0.25f);
        EXPECT_GE(candidate.at(0, 0), -2.0f);
        EXPECT_LE(candidate.at(0, 0), 2.0f);
        EXPECT_GE(candidate.at(1, 0), -2.0f);
        EXPECT_LE(candidate.at(1, 0), 2.0f);
    }
}
