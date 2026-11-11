#include "numerical/motion_profiles/TrapezoidalProfileFromState.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using motion_profiles::AsymmetricMotionLimits;
    using motion_profiles::TrapezoidalProfileFromState;

    class TestTrapezoidalProfileFromState : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        const AsymmetricMotionLimits<float> limits{ 20.0f, 400.0f, 400.0f };

        template<class Metric>
        static float MaxOver(const TrapezoidalProfileFromState<float>& profile, Metric metric)
        {
            float maximum{ -1e30f };
            for (int i = 0; i <= 2000; ++i)
                maximum = std::max(maximum, metric(profile.Sample(profile.Duration() * static_cast<float>(i) / 2000.0f)));

            return maximum;
        }

        static float LargestPositionStep(const TrapezoidalProfileFromState<float>& profile)
        {
            const float step{ profile.Duration() / 2000.0f };
            float largest{ 0.0f };
            for (int i = 1; i <= 2000; ++i)
                largest = std::max(largest, std::abs(profile.Sample(step * static_cast<float>(i)).position - profile.Sample(step * static_cast<float>(i - 1)).position));

            return largest;
        }
    };
}

TEST_F(TestTrapezoidalProfileFromState, from_rest_it_is_the_rest_to_rest_trapezoid)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 0.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Duration(), 0.125f, tolerance);
    EXPECT_NEAR(profile.Sample(0.05f).velocity, 20.0f, 1e-3f);
    EXPECT_NEAR(profile.Sample(0.0625f).position, 0.75f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, it_ends_on_the_goal_at_rest)
{
    const TrapezoidalProfileFromState<float> profile{ 0.3f, -4.0f, 1.5f, limits };

    const auto end{ profile.Sample(profile.Duration()) };

    EXPECT_NEAR(end.position, 1.5f, tolerance);
    EXPECT_NEAR(end.velocity, 0.0f, tolerance);
    EXPECT_NEAR(end.acceleration, 0.0f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, it_starts_from_the_given_position_and_velocity)
{
    const TrapezoidalProfileFromState<float> profile{ 0.3f, 7.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Sample(0.0f).position, 0.3f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).velocity, 7.0f, tolerance);
    EXPECT_NEAR(profile.Sample(-1.0f).position, 0.3f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, a_separate_deceleration_lengthens_the_last_phase)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 0.0f, 2.0f, { 20.0f, 400.0f, 200.0f } };

    EXPECT_NEAR(profile.Duration(), 0.175f, tolerance);
    EXPECT_NEAR(profile.Sample(0.06f).acceleration, 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.15f).acceleration, -200.0f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, a_short_move_peaks_below_the_cruise_speed)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 0.0f, 0.5f, limits };

    EXPECT_NEAR(profile.Duration(), 2.0f * std::sqrt(0.5f / 400.0f), tolerance);
    EXPECT_NEAR(MaxOver(profile, [](const auto& state)
                    {
                        return state.velocity;
                    }),
        std::sqrt(200.0f), 1e-2f);
}

TEST_F(TestTrapezoidalProfileFromState, a_start_towards_the_goal_shortens_the_ramp)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 10.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Sample(0.025f).velocity, 20.0f, 1e-3f);
    EXPECT_NEAR(profile.Sample(0.024f).acceleration, 400.0f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, a_start_away_from_the_goal_turns_round_and_cruises_back)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, -10.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Duration(), 0.15625f, tolerance);
    EXPECT_NEAR(profile.Sample(0.025f).velocity, 0.0f, 1e-3f);
    EXPECT_NEAR(profile.Sample(0.025f).position, -0.125f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, a_start_too_fast_to_stop_overshoots_and_settles_from_beyond)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 20.0f, 0.2f, limits };

    EXPECT_GT(MaxOver(profile, [](const auto& state)
                  {
                      return state.position;
                  }),
        0.2f);
    EXPECT_LT(profile.Sample(profile.Duration() - 0.001f).velocity, 0.0f);
    EXPECT_NEAR(profile.Sample(profile.Duration()).position, 0.2f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, a_start_above_the_cruise_speed_slows_to_it_at_the_deceleration_limit)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 30.0f, 5.0f, { 20.0f, 400.0f, 200.0f } };

    EXPECT_NEAR(profile.Sample(0.0f).acceleration, -200.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.05f).velocity, 20.0f, 1e-3f);
}

TEST_F(TestTrapezoidalProfileFromState, velocity_and_acceleration_stay_within_the_limits)
{
    const TrapezoidalProfileFromState<float> profile{ 0.2f, -6.0f, -3.0f, { 20.0f, 400.0f, 250.0f } };

    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return std::abs(state.velocity);
                  }),
        20.0f + tolerance);
    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return std::abs(state.acceleration);
                  }),
        400.0f + tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, the_position_is_continuous_across_the_phases)
{
    const TrapezoidalProfileFromState<float> profile{ 0.2f, -6.0f, 3.0f, { 20.0f, 400.0f, 250.0f } };

    EXPECT_LE(LargestPositionStep(profile), 20.0f * profile.Duration() / 2000.0f + 1e-4f);
}

TEST_F(TestTrapezoidalProfileFromState, beyond_the_end_it_holds_the_goal_at_rest)
{
    const TrapezoidalProfileFromState<float> profile{ 0.0f, 0.0f, -1.5f, limits };

    const auto held{ profile.Sample(profile.Duration() + 1.0f) };

    EXPECT_NEAR(held.position, -1.5f, tolerance);
    EXPECT_NEAR(held.velocity, 0.0f, tolerance);
}

TEST_F(TestTrapezoidalProfileFromState, at_the_goal_and_at_rest_the_plan_is_empty)
{
    const TrapezoidalProfileFromState<float> profile{ 1.0f, 0.0f, 1.0f, limits };

    EXPECT_NEAR(profile.Duration(), 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).position, 1.0f, tolerance);
}
