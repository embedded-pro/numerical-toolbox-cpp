#include "numerical/filters/active/GravityTilt.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <limits>

namespace
{
    class TestGravityTilt
        : public ::testing::Test
    {
    public:
        static constexpr float g{ 9.81f };
        static constexpr float band{ 0.2f };
        const float tolerance{ math::Tolerance<float>() };

        void ExpectRejected(const filters::GravityTilt<float>& tilt) const
        {
            EXPECT_NEAR(tilt.angle, 0.0f, tolerance);
            EXPECT_NEAR(tilt.trust, 0.0f, tolerance);
        }
    };
}

TEST_F(TestGravityTilt, level_gives_zero_angle_and_full_trust)
{
    const auto tilt = filters::TiltFromGravity(0.0f, g, 0.0f, g, band);

    EXPECT_NEAR(tilt.angle, 0.0f, tolerance);
    EXPECT_NEAR(tilt.trust, 1.0f, tolerance);
}

TEST_F(TestGravityTilt, a_known_tilt_is_recovered)
{
    const auto tilt = filters::TiltFromGravity(g * std::sin(0.3f), g * std::cos(0.3f), 0.0f, g, band);

    EXPECT_NEAR(tilt.angle, 0.3f, tolerance);
    EXPECT_NEAR(tilt.trust, 1.0f, tolerance);
}

TEST_F(TestGravityTilt, a_tilt_the_other_way_is_negative)
{
    EXPECT_NEAR(filters::TiltFromGravity(-g * std::sin(0.3f), g * std::cos(0.3f), 0.0f, g, band).angle, -0.3f, tolerance);
}

TEST_F(TestGravityTilt, the_lateral_component_counts_only_in_the_magnitude)
{
    const auto tilt = filters::TiltFromGravity(0.0f, g * std::cos(0.5f), g * std::sin(0.5f), g, band);

    EXPECT_NEAR(tilt.angle, 0.0f, tolerance);
    EXPECT_NEAR(tilt.trust, 1.0f, tolerance);
}

TEST_F(TestGravityTilt, trust_falls_linearly_with_the_magnitude_error)
{
    EXPECT_NEAR(filters::TiltFromGravity(0.0f, 1.1f * g, 0.0f, g, band).trust, 0.5f, tolerance);
    EXPECT_NEAR(filters::TiltFromGravity(0.0f, 0.9f * g, 0.0f, g, band).trust, 0.5f, tolerance);
}

TEST_F(TestGravityTilt, trust_is_clamped_at_zero)
{
    ExpectRejected(filters::TiltFromGravity(0.0f, 1.5f * g, 0.0f, g, band));
    ExpectRejected(filters::TiltFromGravity(0.0f, 0.0f, 0.0f, g, band));
}

TEST_F(TestGravityTilt, bad_input_is_rejected)
{
    const float nan{ std::numeric_limits<float>::quiet_NaN() };
    const float infinity{ std::numeric_limits<float>::infinity() };

    ExpectRejected(filters::TiltFromGravity(nan, g, 0.0f, g, band));
    ExpectRejected(filters::TiltFromGravity(0.0f, infinity, 0.0f, g, band));
    ExpectRejected(filters::TiltFromGravity(0.0f, g, nan, g, band));
    ExpectRejected(filters::TiltFromGravity(0.0f, g, 0.0f, 0.0f, band));
    ExpectRejected(filters::TiltFromGravity(0.0f, g, 0.0f, g, -0.1f));
}
