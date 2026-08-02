#include "numerical/analysis/Decibels.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    class TestDecibels : public ::testing::Test
    {};
}

TEST_F(TestDecibels, ratio_ten_yields_twenty_db)
{
    EXPECT_NEAR(analysis::ToDecibels(10.0f), 20.0f, math::Tolerance<float>());
}

TEST_F(TestDecibels, ratio_hundred_yields_forty_db)
{
    EXPECT_NEAR(analysis::ToDecibels(100.0f), 40.0f, math::Tolerance<float>());
}

TEST_F(TestDecibels, ratio_half_yields_minus_six_db)
{
    EXPECT_NEAR(analysis::ToDecibels(0.5f), -6.0206f, 1e-3f);
}

TEST_F(TestDecibels, ratio_one_yields_zero_db)
{
    EXPECT_NEAR(analysis::ToDecibels(1.0f), 0.0f, math::Tolerance<float>());
}

TEST_F(TestDecibels, zero_ratio_returns_floor)
{
    EXPECT_NEAR(analysis::ToDecibels(0.0f), analysis::DecibelFloor<float>::value, math::Tolerance<float>());
}

TEST_F(TestDecibels, negative_ratio_returns_floor)
{
    EXPECT_NEAR(analysis::ToDecibels(-1.0f), analysis::DecibelFloor<float>::value, math::Tolerance<float>());
}

TEST_F(TestDecibels, tiny_positive_ratio_clamps_to_floor)
{
    EXPECT_NEAR(analysis::ToDecibels(1e-9f), analysis::DecibelFloor<float>::value, math::Tolerance<float>());
}

TEST_F(TestDecibels, from_decibels_twenty_returns_ten)
{
    EXPECT_NEAR(analysis::FromDecibels(20.0f), 10.0f, math::Tolerance<float>());
}

TEST_F(TestDecibels, round_trip_preserves_ratio)
{
    EXPECT_NEAR(analysis::FromDecibels(analysis::ToDecibels(0.5f)), 0.5f, math::Tolerance<float>());
}

TEST_F(TestDecibels, attenuation_db_computes_difference)
{
    EXPECT_NEAR(analysis::AttenuationDb(1.0f, 0.01f), 40.0f, math::Tolerance<float>());
}

TEST_F(TestDecibels, ripple_db_computes_passband_variation)
{
    EXPECT_NEAR(analysis::RippleDb(1.0f, 0.9f), 0.9151f, 1e-3f);
}
