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

TEST_F(TestSlewLimitedSaturation, clamp_at_exact_lo_boundary)
{
    EXPECT_NEAR(sat.Clamp(-0.5f), -0.5f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, clamp_at_exact_hi_boundary)
{
    EXPECT_NEAR(sat.Clamp(0.5f), 0.5f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, clamp_symmetry_around_zero)
{
    EXPECT_NEAR(sat.Clamp(0.7f), -sat.Clamp(-0.7f), 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_zero_rate_freezes_output)
{
    controllers::RateLimiter<float> frozen{ 0.0f, 1.0f };
    frozen.Limit(1.0f);
    EXPECT_NEAR(frozen.Limit(5.0f), 1.0f, 1e-6f);
    EXPECT_NEAR(frozen.Limit(-3.0f), 1.0f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, reset_default_value_is_zero)
{
    slew.Limit(0.8f);
    slew.Reset();
    EXPECT_NEAR(slew.Limit(0.05f), 0.05f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, reset_produces_same_output_as_fresh_instance)
{
    controllers::RateLimiter<float> fresh{ 0.1f, 1.0f };
    slew.Limit(0.0f);
    slew.Limit(1.0f);
    slew.Reset(0.0f);
    fresh.Limit(0.0f);
    EXPECT_NEAR(slew.Limit(1.0f), fresh.Limit(1.0f), 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, two_instances_do_not_interfere)
{
    controllers::RateLimiter<float> a{ 0.1f, 1.0f };
    controllers::RateLimiter<float> b{ 0.1f, 1.0f };
    a.Limit(0.0f);
    b.Limit(0.0f);
    a.Limit(1.0f);
    EXPECT_NEAR(b.Limit(1.0f), 0.1f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_zero_crossing_negative_to_positive)
{
    controllers::RateLimiter<float> rl{ 0.1f, 1.0f };
    rl.Limit(-0.5f);
    EXPECT_NEAR(rl.Limit(1.0f), -0.4f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, rate_limiter_output_monotonically_approaches_target)
{
    controllers::RateLimiter<float> rl{ 0.1f, 1.0f };
    rl.Limit(0.0f);
    float prev{ 0.0f };
    for (int i = 0; i < 8; ++i)
    {
        float out{ rl.Limit(1.0f) };
        EXPECT_GE(out, prev);
        prev = out;
    }
}

TEST_F(TestSlewLimitedSaturation, slew_limited_saturation_large_negative_input_clamps_at_lo)
{
    controllers::SlewLimitedSaturation<float> composed{ -0.5f, 0.5f, 2.0f, 1.0f };
    composed.Apply(0.0f);
    float out{ composed.Apply(-10.0f) };
    EXPECT_NEAR(out, -0.5f, 1e-6f);
}

TEST_F(TestSlewLimitedSaturation, slew_limited_saturation_determinism)
{
    controllers::SlewLimitedSaturation<float> a{ -0.5f, 0.5f, 0.1f, 1.0f };
    controllers::SlewLimitedSaturation<float> b{ -0.5f, 0.5f, 0.1f, 1.0f };
    std::array<float, 5> inputs{ 0.0f, 1.0f, -1.0f, 0.3f, 0.9f };
    for (float u : inputs)
    {
        EXPECT_FLOAT_EQ(a.Apply(u), b.Apply(u));
    }
}
