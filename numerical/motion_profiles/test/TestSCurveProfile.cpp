#include "numerical/motion_profiles/SCurveProfile.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using motion_profiles::MotionLimits;
    using motion_profiles::SCurveProfile;
    using motion_profiles::TrajectoryState;

    class TestSCurveProfile : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        static constexpr int samples{ 5000 };
        const MotionLimits<float> limits{ 1.0f, 2.0f, 10.0f };
        const SCurveProfile<float> profile{ 0.0f, 5.0f, limits };

        static TrajectoryState<float> SampleAt(const SCurveProfile<float>& p, int i)
        {
            return p.Sample(p.Duration() * static_cast<float>(i) / static_cast<float>(samples));
        }

        template<class Metric>
        static float MaxOver(const SCurveProfile<float>& p, Metric metric)
        {
            float maximum{ 0.0f };
            for (int i = 0; i <= samples; ++i)
                maximum = std::max(maximum, metric(SampleAt(p, i)));

            return maximum;
        }
    };
}

TEST_F(TestSCurveProfile, endpoints_reached_exactly)
{
    EXPECT_NEAR(profile.Sample(0.0f).position, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(profile.Duration()).position, 5.0f, tolerance);
}

TEST_F(TestSCurveProfile, endpoints_at_rest_and_zero_accel)
{
    EXPECT_NEAR(profile.Sample(0.0f).velocity, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).acceleration, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(profile.Duration()).velocity, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(profile.Duration()).acceleration, 0.0f, tolerance);
}

TEST_F(TestSCurveProfile, velocity_never_exceeds_vmax)
{
    EXPECT_LE(MaxOver(profile, [](const auto& s)
                  {
                      return std::abs(s.velocity);
                  }),
        limits.vMax + tolerance);
}

TEST_F(TestSCurveProfile, acceleration_never_exceeds_amax)
{
    EXPECT_LE(MaxOver(profile, [](const auto& s)
                  {
                      return std::abs(s.acceleration);
                  }),
        limits.aMax + tolerance);
}

TEST_F(TestSCurveProfile, jerk_never_exceeds_jmax)
{
    for (int i = 0; i <= samples; ++i)
    {
        const float jerk{ std::abs(SampleAt(profile, i).jerk) };
        EXPECT_TRUE(jerk == 0.0f || jerk == limits.jMax) << "jerk = " << jerk;
    }
}

TEST_F(TestSCurveProfile, acceleration_is_continuous)
{
    const float dt{ profile.Duration() / static_cast<float>(samples) };

    for (int i = 1; i <= samples; ++i)
        EXPECT_LE(std::abs(SampleAt(profile, i).acceleration - SampleAt(profile, i - 1).acceleration), limits.jMax * dt + tolerance);
}

TEST_F(TestSCurveProfile, step1_full_profile_durations)
{
    EXPECT_NEAR(profile.Duration(), 5.7f, tolerance);
    EXPECT_NEAR(profile.Sample(0.2f).acceleration, 2.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.7f).velocity, 1.0f, tolerance);
    EXPECT_TRUE(profile.ReachesMaxAccel());
    EXPECT_TRUE(profile.ReachesMaxVel());
}

TEST_F(TestSCurveProfile, step1_triangular_acceleration_when_jerk_low)
{
    const SCurveProfile<float> lowJerk{ 0.0f, 5.0f, MotionLimits<float>{ 1.0f, 2.0f, 1.0f } };

    EXPECT_NEAR(lowJerk.Duration(), 7.0f, tolerance);
    EXPECT_NEAR(lowJerk.Sample(1.0f).acceleration, 1.0f, tolerance);
    EXPECT_NEAR(lowJerk.Sample(3.5f).velocity, 1.0f, tolerance);
    EXPECT_FALSE(lowJerk.ReachesMaxAccel());
    EXPECT_TRUE(lowJerk.ReachesMaxVel());
}

TEST_F(TestSCurveProfile, step2_short_move_reaches_amax_not_vmax)
{
    const SCurveProfile<float> shortMove{ 0.0f, 0.3f, limits };

    EXPECT_NEAR(shortMove.Duration(), 1.0f, tolerance);
    EXPECT_NEAR(shortMove.Sample(0.5f).velocity, 0.6f, tolerance);
    EXPECT_NEAR(MaxOver(shortMove, [](const auto& s)
                    {
                        return std::abs(s.acceleration);
                    }),
        2.0f, tolerance);
    EXPECT_TRUE(shortMove.ReachesMaxAccel());
    EXPECT_FALSE(shortMove.ReachesMaxVel());
    EXPECT_NEAR(shortMove.Sample(shortMove.Duration()).position, 0.3f, tolerance);
}

TEST_F(TestSCurveProfile, step3_very_short_move_reaches_neither)
{
    const SCurveProfile<float> veryShortMove{ 0.0f, 0.02f, limits };

    EXPECT_NEAR(veryShortMove.Duration(), 0.4f, tolerance);
    EXPECT_NEAR(veryShortMove.Sample(0.1f).acceleration, 1.0f, tolerance);
    EXPECT_NEAR(veryShortMove.Sample(0.2f).velocity, 0.1f, tolerance);
    EXPECT_FALSE(veryShortMove.ReachesMaxAccel());
    EXPECT_FALSE(veryShortMove.ReachesMaxVel());
    EXPECT_NEAR(veryShortMove.Sample(veryShortMove.Duration()).position, 0.02f, tolerance);
}

TEST_F(TestSCurveProfile, negative_direction_mirrors_profile)
{
    const SCurveProfile<float> backwards{ 5.0f, 0.0f, limits };

    for (int i = 0; i <= samples; i += 50)
    {
        const auto forward{ SampleAt(profile, i) };
        const auto mirrored{ SampleAt(backwards, i) };

        EXPECT_NEAR(mirrored.position, 5.0f - forward.position, tolerance);
        EXPECT_NEAR(mirrored.velocity, -forward.velocity, tolerance);
        EXPECT_NEAR(mirrored.acceleration, -forward.acceleration, tolerance);
        EXPECT_NEAR(mirrored.jerk, -forward.jerk, tolerance);
    }
}

TEST_F(TestSCurveProfile, sample_clamps_outside_domain)
{
    EXPECT_EQ(profile.Sample(-1.0f), profile.Sample(0.0f));
    EXPECT_EQ(profile.Sample(profile.Duration() + 1.0f), profile.Sample(profile.Duration()));
}
