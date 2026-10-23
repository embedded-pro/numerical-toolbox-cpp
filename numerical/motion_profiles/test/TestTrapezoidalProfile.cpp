#include "numerical/motion_profiles/TrapezoidalProfile.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using motion_profiles::MotionLimits;
    using motion_profiles::TrapezoidalProfile;

    class TestTrapezoidalProfile : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        const MotionLimits<float> limits{ 1.0f, 2.0f };
        const TrapezoidalProfile<float> profile{ 0.0f, 5.0f, limits };

        template<class Metric>
        static float MaxOver(const TrapezoidalProfile<float>& p, Metric metric)
        {
            float maximum{ -1e30f };
            for (int i = 0; i <= 1000; ++i)
                maximum = std::max(maximum, metric(p.Sample(p.Duration() * static_cast<float>(i) / 1000.0f)));

            return maximum;
        }
    };
}

TEST_F(TestTrapezoidalProfile, endpoints_reached_exactly)
{
    EXPECT_NEAR(profile.Sample(0.0f).position, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(profile.Duration()).position, 5.0f, tolerance);
}

TEST_F(TestTrapezoidalProfile, endpoints_are_at_rest)
{
    EXPECT_NEAR(profile.Sample(0.0f).velocity, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(profile.Duration()).velocity, 0.0f, tolerance);
}

TEST_F(TestTrapezoidalProfile, cruise_velocity_is_clamped_to_vmax)
{
    const float peak{ MaxOver(profile, [](const auto& s)
        {
            return s.velocity;
        }) };

    EXPECT_NEAR(peak, limits.vMax, tolerance);
    EXPECT_LE(peak, limits.vMax);
}

TEST_F(TestTrapezoidalProfile, acceleration_is_within_limit)
{
    EXPECT_LE(MaxOver(profile, [](const auto& s)
                  {
                      return std::abs(s.acceleration);
                  }),
        limits.aMax);
}

TEST_F(TestTrapezoidalProfile, short_move_is_triangular)
{
    const TrapezoidalProfile<float> shortMove{ 0.0f, 0.1f, limits };

    EXPECT_TRUE(shortMove.IsTriangular());
    EXPECT_FALSE(profile.IsTriangular());
    EXPECT_LT(MaxOver(shortMove, [](const auto& s)
                  {
                      return s.velocity;
                  }),
        limits.vMax);
    EXPECT_NEAR(shortMove.Sample(shortMove.Duration()).position, 0.1f, tolerance);
}

TEST_F(TestTrapezoidalProfile, phase_durations_match_trapezoid_formula)
{
    EXPECT_NEAR(profile.Duration(), 5.5f, tolerance);
    EXPECT_NEAR(profile.Sample(0.25f).velocity, 0.5f, tolerance);
    EXPECT_NEAR(profile.Sample(0.5f).velocity, 1.0f, tolerance);
    EXPECT_NEAR(profile.Sample(5.0f).velocity, 1.0f, tolerance);
    EXPECT_NEAR(profile.Sample(5.25f).velocity, 0.5f, tolerance);
}

TEST_F(TestTrapezoidalProfile, negative_direction_move)
{
    const TrapezoidalProfile<float> backwards{ 2.0f, 0.0f, limits };

    EXPECT_LE(MaxOver(backwards, [](const auto& s)
                  {
                      return s.velocity;
                  }),
        0.0f);
    EXPECT_NEAR(backwards.Sample(0.0f).position, 2.0f, tolerance);
    EXPECT_NEAR(backwards.Sample(backwards.Duration()).position, 0.0f, tolerance);
}

TEST_F(TestTrapezoidalProfile, plan_with_duration_lowers_cruise_speed)
{
    const auto planned{ TrapezoidalProfile<float>::PlanWithDuration(0.0f, 4.0f, 1.0f, 5.0f) };

    ASSERT_TRUE(planned.has_value());
    EXPECT_NEAR(planned->Duration(), 5.0f, tolerance);
    EXPECT_NEAR(planned->Sample(0.5f).velocity, 0.5f, tolerance);
    EXPECT_NEAR(planned->Sample(2.5f).velocity, 1.0f, tolerance);
    EXPECT_NEAR(planned->Sample(5.0f).position, 4.0f, tolerance);
}

TEST_F(TestTrapezoidalProfile, plan_with_duration_rejects_infeasible_time)
{
    EXPECT_FALSE(TrapezoidalProfile<float>::PlanWithDuration(0.0f, 4.0f, 1.0f, 3.0f).has_value());
}

TEST_F(TestTrapezoidalProfile, sample_clamps_outside_domain)
{
    EXPECT_EQ(profile.Sample(-1.0f), profile.Sample(0.0f));
    EXPECT_EQ(profile.Sample(profile.Duration() + 1.0f), profile.Sample(profile.Duration()));
}
