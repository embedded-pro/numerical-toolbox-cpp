#include "numerical/math/Tolerance.hpp"
#include "numerical/optimization/BayesianOptimization.hpp"
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

        class QuadraticObjective : public Bo::BlackBoxObjective
        {
        public:
            float Evaluate(const ParamVec& params) override
            {
                const float x = params.at(0, 0);
                return x * x;
            }
        };

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

        class BowlObjective : public Bo::BlackBoxObjective
        {
        public:
            float Evaluate(const ParamVec& params) override
            {
                const float dx = params.at(0, 0) - 0.4f;
                const float dy = params.at(1, 0) + 0.3f;
                return dx * dx + dy * dy;
            }
        };

        optimization::GpHyperparameters hp{ 0.8f, 1.0f, 0.01f };
        Bo::BoundsArray bounds{
            std::make_pair(-2.0f, 2.0f),
            std::make_pair(-2.0f, 2.0f)
        };
        Bo bo{ bounds, hp, 12345ULL };
    };

    class TestBayesianOptimizationCapacity : public ::testing::Test
    {
    protected:
        static constexpr std::size_t NumParams = 1;
        static constexpr std::size_t MaxObs = 4;
        static constexpr std::size_t NumCandidates = 10;

        using Bo = optimization::BayesianOptimization<NumParams, MaxObs, NumCandidates>;
        using ParamVec = Bo::ParameterVector;

        optimization::GpHyperparameters hp{ 0.5f, 1.0f, 0.01f };
        Bo::BoundsArray bounds{ std::make_pair(-1.0f, 1.0f) };
        Bo bo{ bounds, hp, 7777ULL };
    };
}

TEST_F(TestBayesianOptimization1D, get_num_observations_starts_at_zero)
{
    EXPECT_EQ(bo.GetNumObservations(), 0u);
}

TEST_F(TestBayesianOptimization1D, get_num_observations_increments_per_add)
{
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);
    EXPECT_EQ(bo.GetNumObservations(), 1u);
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);
    EXPECT_EQ(bo.GetNumObservations(), 2u);
}

TEST_F(TestBayesianOptimization1D, get_observed_values_reflects_added_observations)
{
    bo.AddObservation(ParamVec{ { -1.0f } }, 2.5f);
    bo.AddObservation(ParamVec{ { 0.5f } }, 0.25f);

    const auto& vals = bo.GetObservedValues();
    EXPECT_FLOAT_EQ(vals[0], 2.5f);
    EXPECT_FLOAT_EQ(vals[1], 0.25f);
}

TEST_F(TestBayesianOptimization1D, gp_mean_at_observed_point_matches_noise_free_posterior)
{
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);

    const auto [mu, sigma] = bo.GpPredict(ParamVec{ { 1.0f } });

    EXPECT_NEAR(mu, 1.0f, 1e-3f);
    EXPECT_GE(sigma, 0.0f);
}

TEST_F(TestBayesianOptimization1D, gp_mean_at_three_observed_points_matches_reference)
{
    bo.AddObservation(ParamVec{ { -1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);

    const auto [mu0, sigma0] = bo.GpPredict(ParamVec{ { 0.0f } });
    const auto [mu1, sigma1] = bo.GpPredict(ParamVec{ { 1.0f } });

    EXPECT_NEAR(mu0, 0.0f, 1e-3f);
    EXPECT_NEAR(mu1, 1.0f, 1e-3f);
    EXPECT_GE(sigma0, 0.0f);
    EXPECT_GE(sigma1, 0.0f);
}

TEST_F(TestBayesianOptimization1D, expected_improvement_is_zero_before_observations)
{
    const float ei = bo.ExpectedImprovement(ParamVec{ { 0.5f } }, 0.0f);

    EXPECT_FLOAT_EQ(ei, 0.0f);
}

TEST_F(TestBayesianOptimization1D, expected_improvement_is_strictly_positive_far_from_observation)
{
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);

    const float ei = bo.ExpectedImprovement(ParamVec{ { 2.5f } }, 0.0f);

    EXPECT_GT(ei, 0.3f);
}

TEST_F(TestBayesianOptimization1D, expected_improvement_is_nonnegative_near_observation)
{
    bo.AddObservation(ParamVec{ { -1.0f } }, 1.0f);
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.0f);

    const float ei = bo.ExpectedImprovement(ParamVec{ { 2.0f } }, 0.0f);

    EXPECT_GE(ei, 0.0f);
}

TEST_F(TestBayesianOptimization1D, find_best_index_reflects_lower_valued_observation)
{
    bo.AddObservation(ParamVec{ { 1.0f } }, 2.0f);
    bo.AddObservation(ParamVec{ { -1.0f } }, 0.5f);

    const auto& vals = bo.GetObservedValues();
    EXPECT_FLOAT_EQ(vals[0], 2.0f);
    EXPECT_FLOAT_EQ(vals[1], 0.5f);

    bo.AddObservation(ParamVec{ { 0.5f } }, 1.5f);
    const float bestVal = bo.GetObservedValues()[1];
    EXPECT_FLOAT_EQ(bestVal, 0.5f);
}

TEST_F(TestBayesianOptimization1D, optimize_evaluations_equals_num_iterations)
{
    QuadraticObjective objective;

    const auto result = bo.Optimize(objective, 8);

    EXPECT_EQ(result.evaluations, 8u);
}

TEST_F(TestBayesianOptimization1D, optimize_1d_quadratic_finds_point_near_zero)
{
    QuadraticObjective objective;

    const auto result = bo.Optimize(objective, 8);

    EXPECT_LT(std::abs(result.bestPoint.at(0, 0)), 0.5f);
}

TEST_F(TestBayesianOptimization1D, optimize_best_value_is_quadratic_of_best_point)
{
    QuadraticObjective objective;

    const auto result = bo.Optimize(objective, 8);
    const float x = result.bestPoint.at(0, 0);

    EXPECT_NEAR(result.bestValue, x * x, math::Tolerance<float>());
}

TEST_F(TestBayesianOptimization1D, maximize_acquisition_returns_point_within_bounds)
{
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.1f);
    bo.AddObservation(ParamVec{ { 1.0f } }, 1.0f);

    const auto candidate = bo.MaximizeAcquisition(0.1f);

    EXPECT_GE(candidate.at(0, 0), -3.0f);
    EXPECT_LE(candidate.at(0, 0), 3.0f);
}

TEST_F(TestBayesianOptimization1D, determinism_same_seed_produces_identical_results)
{
    Bo bo2{ bounds, hp, 99991ULL };
    QuadraticObjective objective;

    const auto result1 = bo.Optimize(objective, 7);
    const auto result2 = bo2.Optimize(objective, 7);

    EXPECT_FLOAT_EQ(result1.bestPoint.at(0, 0), result2.bestPoint.at(0, 0));
    EXPECT_FLOAT_EQ(result1.bestValue, result2.bestValue);
}

TEST_F(TestBayesianOptimization1D, different_seeds_may_differ)
{
    Bo bo_alt{ bounds, hp, 11111ULL };
    QuadraticObjective objective;

    const auto result1 = bo.Optimize(objective, 7);
    const auto result2 = bo_alt.Optimize(objective, 7);

    const bool pointsDiffer =
        result1.bestPoint.at(0, 0) != result2.bestPoint.at(0, 0) ||
        result1.bestValue != result2.bestValue;
    EXPECT_TRUE(pointsDiffer);
}

TEST_F(TestBayesianOptimizationCapacity, filling_to_capacity_does_not_crash)
{
    bo.AddObservation(ParamVec{ { -0.5f } }, 0.8f);
    bo.AddObservation(ParamVec{ { 0.0f } }, 0.1f);
    bo.AddObservation(ParamVec{ { 0.5f } }, 0.4f);
    bo.AddObservation(ParamVec{ { -0.3f } }, 0.3f);

    EXPECT_EQ(bo.GetNumObservations(), MaxObs);

    const auto& vals = bo.GetObservedValues();
    EXPECT_FLOAT_EQ(vals[0], 0.8f);
    EXPECT_FLOAT_EQ(vals[1], 0.1f);
    EXPECT_FLOAT_EQ(vals[2], 0.4f);
    EXPECT_FLOAT_EQ(vals[3], 0.3f);
}

TEST_F(TestBayesianOptimization2D, optimize_2d_bowl_finds_point_near_known_min)
{
    BowlObjective objective;

    const auto result = bo.Optimize(objective, 18);

    const float dx = result.bestPoint.at(0, 0) - 0.4f;
    const float dy = result.bestPoint.at(1, 0) + 0.3f;
    EXPECT_LT(std::sqrt(dx * dx + dy * dy), 0.6f);
}

TEST_F(TestBayesianOptimization2D, optimize_evaluations_equals_num_iterations_2d)
{
    BowlObjective objective;

    const auto result = bo.Optimize(objective, 18);

    EXPECT_EQ(result.evaluations, 18u);
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

TEST_F(TestBayesianOptimization2D, gp_predict_finite_and_nonneg_std)
{
    bo.AddObservation(ParamVec{ { -1.0f }, { -1.0f } }, 2.0f);
    bo.AddObservation(ParamVec{ { 0.0f }, { 0.0f } }, 0.25f);
    bo.AddObservation(ParamVec{ { 1.0f }, { 1.0f } }, 2.0f);

    const auto [mu, sigma] = bo.GpPredict(ParamVec{ { 0.5f }, { 0.5f } });

    EXPECT_TRUE(std::isfinite(mu));
    EXPECT_GE(sigma, 0.0f);
}
