#include "numerical/math/FastTrigonometry.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <numbers>

namespace
{
    constexpr float pi = std::numbers::pi_v<float>;
    constexpr float lutTolerance = 0.015f;
    constexpr float accurateTolerance = 2e-6f;

    class TestFastTrigonometry
        : public ::testing::Test
    {
    };

}

TEST_F(TestFastTrigonometry, sine_zero)
{
    EXPECT_NEAR(math::FastSine(0.0f), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_pi_over_2)
{
    EXPECT_NEAR(math::FastSine(pi / 2.0f), 1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_pi)
{
    EXPECT_NEAR(math::FastSine(pi), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_three_pi_over_2)
{
    EXPECT_NEAR(math::FastSine(3.0f * pi / 2.0f), -1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, cosine_zero)
{
    EXPECT_NEAR(math::FastCosine(0.0f), 1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, cosine_pi_over_2)
{
    EXPECT_NEAR(math::FastCosine(pi / 2.0f), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, cosine_pi)
{
    EXPECT_NEAR(math::FastCosine(pi), -1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_squared_plus_cosine_squared_equals_one)
{
    for (float angle = 0.0f; angle < 2.0f * pi; angle += 0.1f)
    {
        auto s = math::FastSine(angle);
        auto c = math::FastCosine(angle);
        EXPECT_NEAR(s * s + c * c, 1.0f, lutTolerance);
    }
}

TEST_F(TestFastTrigonometry, sine_matches_std_sin_across_range)
{
    for (float angle = 0.0f; angle < 2.0f * pi; angle += 0.05f)
    {
        EXPECT_NEAR(math::FastSine(angle), std::sin(angle), lutTolerance)
            << "at angle " << angle;
    }
}

TEST_F(TestFastTrigonometry, cosine_matches_std_cos_across_range)
{
    for (float angle = 0.0f; angle < 2.0f * pi; angle += 0.05f)
    {
        EXPECT_NEAR(math::FastCosine(angle), std::cos(angle), lutTolerance)
            << "at angle " << angle;
    }
}

TEST_F(TestFastTrigonometry, accurate_sine_and_cosine_match_std_to_float_precision)
{
    for (float angle = -2.0f * pi; angle < 2.0f * pi; angle += 0.0137f)
    {
        const auto result = math::AccurateSineAndCosine(angle);
        EXPECT_NEAR(result.sine, std::sin(angle), accurateTolerance) << "at angle " << angle;
        EXPECT_NEAR(result.cosine, std::cos(angle), accurateTolerance) << "at angle " << angle;
    }
}

TEST_F(TestFastTrigonometry, sine_negative_pi_over_2)
{
    EXPECT_NEAR(math::FastSine(-pi / 2.0f), -1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_negative_pi)
{
    EXPECT_NEAR(math::FastSine(-pi), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_negative_two_pi)
{
    EXPECT_NEAR(math::FastSine(-2.0f * pi), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, cosine_negative_pi_over_2)
{
    EXPECT_NEAR(math::FastCosine(-pi / 2.0f), 0.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, cosine_negative_pi)
{
    EXPECT_NEAR(math::FastCosine(-pi), -1.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_large_positive_angle)
{
    EXPECT_NEAR(math::FastSine(100.0f * pi + pi / 6.0f), 0.5f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_large_negative_angle)
{
    EXPECT_NEAR(math::FastSine(-100.0f * pi - pi / 3.0f), -std::sqrt(3.0f) / 2.0f, lutTolerance);
}

TEST_F(TestFastTrigonometry, sine_matches_std_sin_negative_range)
{
    for (float angle = -4.0f * pi; angle < 0.0f; angle += 0.05f)
    {
        EXPECT_NEAR(math::FastSine(angle), std::sin(angle), lutTolerance)
            << "at angle " << angle;
    }
}

TEST_F(TestFastTrigonometry, cosine_matches_std_cos_negative_range)
{
    for (float angle = -4.0f * pi; angle < 0.0f; angle += 0.05f)
    {
        EXPECT_NEAR(math::FastCosine(angle), std::cos(angle), lutTolerance)
            << "at angle " << angle;
    }
}

TEST_F(TestFastTrigonometry, identity_holds_for_negative_angles)
{
    for (float angle = -2.0f * pi; angle < 0.0f; angle += 0.1f)
    {
        auto s = math::FastSine(angle);
        auto c = math::FastCosine(angle);
        EXPECT_NEAR(s * s + c * c, 1.0f, lutTolerance)
            << "at angle " << angle;
    }
}
