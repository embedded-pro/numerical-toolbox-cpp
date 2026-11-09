#include "numerical/estimators/online/RunningLinearFit.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <cstdint>

namespace
{
    constexpr float slope{ 0.0046f };

    class TestRunningLinearFit
        : public testing::Test
    {
    public:
        void AddLine(float offset, float noise = 0.0f)
        {
            for (int32_t sample = 0; sample != 200; ++sample)
            {
                const float x{ 40.0f + 0.6f * static_cast<float>(sample) };
                const float sign{ (sample % 2 == 0) ? 1.0f : -1.0f };
                fit.Add(x, slope * x + offset + sign * noise);
            }
        }

        estimators::RunningLinearFit<float> fit;
    };
}

TEST_F(TestRunningLinearFit, an_exact_line_gives_its_slope_and_intercept)
{
    AddLine(0.05f);

    EXPECT_EQ(200u, fit.Count());
    EXPECT_NEAR(slope, fit.Slope(), 1e-6f);
    EXPECT_NEAR(0.05f, fit.Intercept(), 1e-4f);
    EXPECT_NEAR(0.0f, fit.RelativeSlopeError(), 1e-3f);
}

TEST_F(TestRunningLinearFit, noise_widens_the_slope_error)
{
    AddLine(0.0f, 0.05f);

    EXPECT_NEAR(slope, fit.Slope(), 5e-5f);
    EXPECT_GT(fit.RelativeSlopeError(), 0.005f);
}

TEST_F(TestRunningLinearFit, a_single_abscissa_has_no_slope)
{
    for (int32_t sample = 0; sample != 10; ++sample)
        fit.Add(80.0f, 0.4f);

    EXPECT_EQ(0.0f, fit.Slope());
    EXPECT_TRUE(std::isinf(fit.RelativeSlopeError()));
}

TEST_F(TestRunningLinearFit, two_points_give_a_slope_but_no_error_estimate)
{
    fit.Add(1.0f, 2.0f);
    fit.Add(3.0f, 6.0f);

    EXPECT_NEAR(2.0f, fit.Slope(), 1e-6f);
    EXPECT_TRUE(std::isinf(fit.RelativeSlopeError()));
}

TEST_F(TestRunningLinearFit, a_flat_line_has_no_relative_error)
{
    for (int32_t sample = 0; sample != 10; ++sample)
        fit.Add(static_cast<float>(sample), 3.0f);

    EXPECT_EQ(0.0f, fit.Slope());
    EXPECT_TRUE(std::isinf(fit.RelativeSlopeError()));
}

TEST_F(TestRunningLinearFit, means_and_sums_of_deviations_match_the_data)
{
    fit.Add(1.0f, 1.0f);
    fit.Add(2.0f, 3.0f);
    fit.Add(3.0f, 2.0f);

    EXPECT_NEAR(2.0f, fit.MeanX(), 1e-6f);
    EXPECT_NEAR(2.0f, fit.MeanY(), 1e-6f);
    EXPECT_NEAR(2.0f, fit.SumSquaredDeviationsX(), 1e-6f);
    EXPECT_NEAR(1.0f, fit.SumCrossDeviations(), 1e-6f);
    EXPECT_NEAR(3.0f, fit.Weight(), 1e-6f);
}

TEST_F(TestRunningLinearFit, a_weighted_point_counts_as_repeated_points)
{
    estimators::RunningLinearFit<float> repeated;
    fit.Add(1.0f, 1.0f, 3.0f);
    fit.Add(4.0f, 2.0f);
    for (int32_t i = 0; i != 3; ++i)
        repeated.Add(1.0f, 1.0f);
    repeated.Add(4.0f, 2.0f);

    EXPECT_NEAR(repeated.MeanX(), fit.MeanX(), 1e-6f);
    EXPECT_NEAR(repeated.SumSquaredDeviationsX(), fit.SumSquaredDeviationsX(), 1e-5f);
    EXPECT_NEAR(repeated.Slope(), fit.Slope(), 1e-6f);
    EXPECT_EQ(2u, fit.Count());
}

TEST_F(TestRunningLinearFit, decay_scales_the_weight_and_the_sums_but_not_the_means)
{
    AddLine(0.0f);
    const auto meanX{ fit.MeanX() };
    const auto sumSquares{ fit.SumSquaredDeviationsX() };

    fit.Decay(0.5f);

    EXPECT_NEAR(100.0f, fit.Weight(), 1e-3f);
    EXPECT_NEAR(meanX, fit.MeanX(), 1e-5f);
    EXPECT_NEAR(0.5f * sumSquares, fit.SumSquaredDeviationsX(), 1e-2f);
}

TEST_F(TestRunningLinearFit, decaying_to_nothing_forgets_the_old_line)
{
    for (int32_t x = 0; x != 10; ++x)
        fit.Add(static_cast<float>(x), 5.0f * static_cast<float>(x));

    fit.Decay(0.0f);
    for (int32_t x = 0; x != 10; ++x)
        fit.Add(static_cast<float>(x), -2.0f * static_cast<float>(x) + 1.0f);

    EXPECT_NEAR(-2.0f, fit.Slope(), 1e-5f);
    EXPECT_NEAR(1.0f, fit.Intercept(), 1e-4f);
}

TEST_F(TestRunningLinearFit, reset_starts_over)
{
    AddLine(0.05f);

    fit.Reset();

    EXPECT_EQ(0u, fit.Count());
    EXPECT_EQ(0.0f, fit.Weight());
    EXPECT_EQ(0.0f, fit.Slope());
}
