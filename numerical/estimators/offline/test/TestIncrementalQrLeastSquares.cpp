#include "numerical/estimators/offline/IncrementalQrLeastSquares.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Line = estimators::IncrementalQrLeastSquares<float, 2>;
    using Redundant = estimators::IncrementalQrLeastSquares<float, 3>;

    class TestIncrementalQrLeastSquares : public ::testing::Test
    {
    protected:
        Line line;
        Redundant redundant;
        const std::array<float, 4> x{ 0.0f, 1.0f, 2.0f, 3.0f };
        const std::array<float, 4> noisy{ 2.1f, 4.9f, 8.1f, 10.9f };

        void AddNoisyRows(Line& estimator, bool reversed) const
        {
            for (std::size_t n = 0; n < x.size(); ++n)
            {
                const std::size_t i{ reversed ? x.size() - 1 - n : n };
                estimator.AddRow(Line::Regressor{ 1.0f, x[i] }, noisy[i]);
            }
        }
    };
}

TEST_F(TestIncrementalQrLeastSquares, exact_data_recovers_parameters)
{
    for (float xi : x)
        line.AddRow(Line::Regressor{ 1.0f, xi }, 2.0f + 3.0f * xi);

    const auto result{ line.Solve(1e-5f) };

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->parameters[0], 2.0f, 1e-5f);
    EXPECT_NEAR(result->parameters[1], 3.0f, 1e-5f);
    EXPECT_EQ(result->rank, 2);
    EXPECT_NEAR(result->residualRms, 0.0f, 1e-5f);
}

TEST_F(TestIncrementalQrLeastSquares, noisy_data_matches_batch_least_squares)
{
    AddNoisyRows(line, false);

    const auto result{ line.Solve(1e-5f) };

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->parameters[0], 2.06f, 1e-5f);
    EXPECT_NEAR(result->parameters[1], 2.96f, 1e-5f);
    EXPECT_NEAR(result->residualRms, 0.089443f, 1e-5f);
}

TEST_F(TestIncrementalQrLeastSquares, rank_deficient_regressor_returns_minimum_norm_solution)
{
    for (float xi : x)
        redundant.AddRow(Redundant::Regressor{ 1.0f, xi, xi }, 2.0f + 3.0f * xi);

    const auto result{ redundant.Solve(1e-5f) };

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->rank, 2);
    EXPECT_NEAR(result->parameters[0], 2.0f, 1e-5f);
    EXPECT_NEAR(result->parameters[1], 1.5f, 1e-5f);
    EXPECT_NEAR(result->parameters[2], 1.5f, 1e-5f);
    EXPECT_NEAR(result->rowSpace.at(2, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(std::abs(result->rowSpace.at(2, 1)), 1.0f / std::sqrt(2.0f), 1e-5f);
    EXPECT_NEAR(result->rowSpace.at(2, 1), -result->rowSpace.at(2, 2), 1e-5f);
}

TEST_F(TestIncrementalQrLeastSquares, row_order_does_not_change_the_result)
{
    Line reversed;
    AddNoisyRows(line, false);
    AddNoisyRows(reversed, true);

    const auto forward{ line.Solve(1e-5f) };
    const auto backward{ reversed.Solve(1e-5f) };

    ASSERT_TRUE(forward.has_value());
    ASSERT_TRUE(backward.has_value());
    EXPECT_NEAR(forward->parameters[0], backward->parameters[0], 1e-5f);
    EXPECT_NEAR(forward->parameters[1], backward->parameters[1], 1e-5f);
    EXPECT_NEAR(forward->residualRms, backward->residualRms, 1e-5f);
}

TEST_F(TestIncrementalQrLeastSquares, block_rows_equal_single_rows)
{
    Line blocked;
    const math::Matrix<float, 4, 2> block{ { 1.0f, x[0] }, { 1.0f, x[1] }, { 1.0f, x[2] }, { 1.0f, x[3] } };
    blocked.AddRows(block, math::Vector<float, 4>{ noisy[0], noisy[1], noisy[2], noisy[3] });
    AddNoisyRows(line, false);

    const auto single{ line.Solve(1e-5f) };
    const auto batch{ blocked.Solve(1e-5f) };

    ASSERT_TRUE(single.has_value());
    ASSERT_TRUE(batch.has_value());
    EXPECT_EQ(blocked.Rows(), 4);
    EXPECT_FLOAT_EQ(batch->parameters[0], single->parameters[0]);
    EXPECT_FLOAT_EQ(batch->parameters[1], single->parameters[1]);
    EXPECT_FLOAT_EQ(batch->residualRms, single->residualRms);
}

TEST_F(TestIncrementalQrLeastSquares, no_rows_returns_nullopt)
{
    EXPECT_FALSE(line.Solve(1e-5f).has_value());

    AddNoisyRows(line, false);
    line.Reset();

    EXPECT_EQ(line.Rows(), 0);
    EXPECT_FALSE(line.Solve(1e-5f).has_value());
}
