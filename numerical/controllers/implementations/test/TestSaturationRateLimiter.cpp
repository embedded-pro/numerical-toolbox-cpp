#include "numerical/controllers/implementations/SaturationRateLimiter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestSlewLimitedSaturation
        : public ::testing::Test
    {
    public:
        controllers::Saturation<float> sat{ -0.5f, 0.5f };
        controllers::RateLimiter<float> slew{ 0.1f, 1.0f };
    };
}

TEST_F(TestSlewLimitedSaturation, clamp_passes_value_inside_bounds)
{
    EXPECT_NEAR(sat.Clamp(0.25f), 0.25f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, clamp_saturates_above_and_below)
{
    EXPECT_NEAR(sat.Clamp(0.9f), 0.5f, 1e-6f);
    EXPECT_NEAR(sat.Clamp(-0.9f), -0.5f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_first_sample_seeds_state)
{
    EXPECT_NEAR(slew.Limit(0.8f), 0.8f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_caps_positive_slope)
{
    slew.Limit(0.0f);
    EXPECT_NEAR(slew.Limit(1.0f), 0.1f, 1e-6f);
    EXPECT_NEAR(slew.Limit(1.0f), 0.2f, 1e-6f);
    EXPECT_NEAR(slew.Limit(1.0f), 0.3f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_caps_negative_slope)
{
    slew.Limit(0.0f);
    EXPECT_NEAR(slew.Limit(-1.0f), -0.1f, 1e-6f);
    EXPECT_NEAR(slew.Limit(-1.0f), -0.2f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_follows_small_changes_exactly)
{
    slew.Limit(0.0f);
    EXPECT_NEAR(slew.Limit(0.05f), 0.05f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, reset_reprimes_state)
{
    slew.Limit(0.0f);
    slew.Limit(1.0f);
    slew.Reset(0.2f);
    EXPECT_NEAR(slew.Limit(0.25f), 0.25f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, composition_slew_then_clamp_honours_both)
{
    controllers::SlewLimitedSaturation<float> composed{ -0.5f, 0.5f, 0.1f, 1.0f };
    composed.Apply(0.0f);
    for (int i = 0; i < 10; ++i)
    {
        float out{ composed.Apply(10.0f) };
        EXPECT_LE(out, 0.5f);
        EXPECT_GE(out, -0.5f);
    }
}
