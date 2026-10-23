#include "numerical/math/AngleUtilities.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestAngleUtilities : public ::testing::Test
    {
    protected:
        static constexpr float pi{ std::numbers::pi_v<float> };
        static constexpr float tolerance{ 1e-5f };

        static constexpr float Degrees(float degrees)
        {
            return degrees * pi / 180.0f;
        }
    };
}

TEST_F(TestAngleUtilities, wrap_to_pi_maps_into_half_open_interval)
{
    EXPECT_NEAR(math::WrapToPi(1.5f * pi), -0.5f * pi, tolerance);
    EXPECT_NEAR(math::WrapToPi(-1.5f * pi), 0.5f * pi, tolerance);
    EXPECT_EQ(math::WrapToPi(0.0f), 0.0f);
    EXPECT_NEAR(math::WrapToPi(7.0f * pi / 3.0f), pi / 3.0f, tolerance);
}

TEST_F(TestAngleUtilities, wrap_to_pi_sends_both_half_turns_to_plus_pi)
{
    EXPECT_NEAR(math::WrapToPi(pi), pi, tolerance);
    EXPECT_NEAR(math::WrapToPi(-pi), pi, tolerance);
}

TEST_F(TestAngleUtilities, wrap_to_two_pi_maps_into_zero_to_two_pi)
{
    EXPECT_NEAR(math::WrapToTwoPi(-0.5f * pi), 1.5f * pi, tolerance);
    EXPECT_NEAR(math::WrapToTwoPi(2.0f * pi), 0.0f, tolerance);
    EXPECT_NEAR(math::WrapToTwoPi(5.0f * pi), pi, 4.0f * tolerance);
    EXPECT_LT(math::WrapToTwoPi(-1e-8f), 2.0f * pi);
}

TEST_F(TestAngleUtilities, angle_difference_takes_the_short_way)
{
    EXPECT_NEAR(math::AngleDifference(Degrees(170.0f), Degrees(-170.0f)), Degrees(-20.0f), tolerance);
    EXPECT_NEAR(math::AngleDifference(Degrees(-170.0f), Degrees(170.0f)), Degrees(20.0f), tolerance);
}

TEST_F(TestAngleUtilities, unwrap_follows_a_continuous_rotation)
{
    float unwrapped{ math::WrapToPi(0.0f) };

    for (int k = 1; k <= 40; ++k)
    {
        const float angle{ 0.3f * static_cast<float>(k) };
        unwrapped = math::UnwrapAngle(unwrapped, math::WrapToPi(angle));

        EXPECT_NEAR(unwrapped, angle, tolerance);
    }
}
