#include "numerical/motion_profiles/SCurveProfileFromState.hpp"
#include "numerical/motion_profiles/TrapezoidalProfileFromState.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using motion_profiles::JerkLimitedMotionLimits;
    using motion_profiles::SCurveProfileFromState;
    using motion_profiles::TrajectoryState;

    class TestSCurveProfileFromState : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-4f };
        static constexpr int samples{ 20000 };
        const JerkLimitedMotionLimits<float> limits{ 20.0f, 400.0f, 400.0f, 20000.0f };
        const JerkLimitedMotionLimits<float> asymmetric{ 20.0f, 400.0f, 250.0f, 20000.0f };

        static TrajectoryState<float> SampleAt(const SCurveProfileFromState<float>& profile, int i)
        {
            return profile.Sample(profile.Duration() * static_cast<float>(i) / static_cast<float>(samples));
        }

        template<class Metric>
        static float MaxOver(const SCurveProfileFromState<float>& profile, Metric metric)
        {
            float maximum{ -1e30f };
            for (int i = 0; i <= samples; ++i)
                maximum = std::max(maximum, metric(SampleAt(profile, i)));

            return maximum;
        }

        template<class Quantity>
        static float LargestStep(const SCurveProfileFromState<float>& profile, Quantity quantity)
        {
            float largest{ 0.0f };
            for (int i = 1; i <= samples; ++i)
                largest = std::max(largest, std::abs(quantity(SampleAt(profile, i)) - quantity(SampleAt(profile, i - 1))));

            return largest;
        }
    };
}

TEST_F(TestSCurveProfileFromState, from_rest_it_is_the_symmetric_double_s)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 0.0f, 0.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Duration(), 0.07f + 0.005f + 0.07f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0725f).position, 0.75f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0725f).velocity, 20.0f, 1e-3f);
}

TEST_F(TestSCurveProfileFromState, it_ends_on_the_goal_at_rest_with_zero_acceleration)
{
    const SCurveProfileFromState<float> profile{ 0.3f, -4.0f, 100.0f, 1.5f, limits };

    const auto end{ profile.Sample(profile.Duration() - 1e-6f) };

    EXPECT_NEAR(end.position, 1.5f, tolerance);
    EXPECT_NEAR(end.velocity, 0.0f, 1e-2f);
    EXPECT_NEAR(end.acceleration, 0.0f, 0.1f);
}

TEST_F(TestSCurveProfileFromState, it_starts_from_the_given_position_velocity_and_acceleration)
{
    const SCurveProfileFromState<float> profile{ 0.3f, 7.0f, -150.0f, 1.5f, limits };

    EXPECT_NEAR(profile.Sample(-1.0f).position, 0.3f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).velocity, 7.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).acceleration, -150.0f, tolerance);
}

TEST_F(TestSCurveProfileFromState, a_start_in_an_acceleration_phase_ramps_the_acceleration_at_the_jerk_limit)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 15.0f, 300.0f, 0.4f, limits };

    EXPECT_NEAR(profile.Sample(0.0f).jerk, -20000.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.001f).acceleration, 280.0f, 1e-2f);
}

TEST_F(TestSCurveProfileFromState, position_velocity_and_acceleration_are_continuous_across_the_phases)
{
    const SCurveProfileFromState<float> profile{ 0.2f, -6.0f, 250.0f, 3.0f, asymmetric };
    const float step{ profile.Duration() / static_cast<float>(samples) };

    EXPECT_LE(LargestStep(profile, [](const auto& state)
                  {
                      return state.position;
                  }),
        20.0f * step + 1e-4f);
    EXPECT_LE(LargestStep(profile, [](const auto& state)
                  {
                      return state.velocity;
                  }),
        400.0f * step + 1e-3f);
    EXPECT_LE(LargestStep(profile, [](const auto& state)
                  {
                      return state.acceleration;
                  }),
        20000.0f * step + 1e-2f);
}

TEST_F(TestSCurveProfileFromState, velocity_acceleration_and_jerk_stay_within_the_limits)
{
    const SCurveProfileFromState<float> profile{ 0.2f, -6.0f, 0.0f, 3.0f, asymmetric };

    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return std::abs(state.velocity);
                  }),
        20.0f + 1e-3f);
    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return state.acceleration * state.velocity > 0.0f ? std::abs(state.acceleration) : 0.0f;
                  }),
        400.0f + 1e-2f);
    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return state.acceleration * state.velocity < 0.0f ? std::abs(state.acceleration) : 0.0f;
                  }),
        250.0f + 1e-2f);
    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return std::abs(state.jerk);
                  }),
        20000.0f);
}

TEST_F(TestSCurveProfileFromState, a_short_move_reaches_neither_the_cruise_speed_nor_the_acceleration_limit)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 0.0f, 0.0f, 0.1f, limits };

    EXPECT_NEAR(profile.Duration(), 4.0f * std::cbrt(0.1f / (2.0f * 20000.0f)), tolerance);
    EXPECT_NEAR(MaxOver(profile, [](const auto& state)
                    {
                        return state.acceleration;
                    }),
        20000.0f * std::cbrt(0.1f / (2.0f * 20000.0f)), 0.5f);
}

TEST_F(TestSCurveProfileFromState, a_start_away_from_the_goal_turns_round_and_reaches_it)
{
    const SCurveProfileFromState<float> profile{ 0.0f, -10.0f, 0.0f, 1.5f, limits };

    EXPECT_LT(-MaxOver(profile, [](const auto& state)
                  {
                      return -state.position;
                  }),
        -0.1f);
    EXPECT_GT(profile.Duration(), 0.15625f);
    EXPECT_NEAR(profile.Sample(profile.Duration() - 1e-6f).position, 1.5f, tolerance);
}

TEST_F(TestSCurveProfileFromState, a_start_too_fast_to_stop_overshoots_and_settles_from_beyond)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 20.0f, 0.0f, 0.2f, limits };

    EXPECT_GT(MaxOver(profile, [](const auto& state)
                  {
                      return state.position;
                  }),
        0.5f);
    EXPECT_LT(profile.Sample(profile.Duration() - 0.01f).velocity, 0.0f);
    EXPECT_NEAR(profile.Sample(profile.Duration() - 1e-6f).position, 0.2f, tolerance);
}

TEST_F(TestSCurveProfileFromState, a_start_above_the_cruise_speed_slows_to_it_at_the_deceleration_limit)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 30.0f, 0.0f, 5.0f, { 20.0f, 400.0f, 200.0f, 20000.0f } };

    EXPECT_NEAR(profile.Sample(0.02f).acceleration, -200.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.08f).velocity, 20.0f, 1e-3f);
    EXPECT_LE(MaxOver(profile, [](const auto& state)
                  {
                      return state.velocity;
                  }),
        30.0f);
}

TEST_F(TestSCurveProfileFromState, a_separate_deceleration_lengthens_the_last_ramp)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 0.0f, 0.0f, 2.0f, { 20.0f, 400.0f, 200.0f, 20000.0f } };

    EXPECT_NEAR(profile.Duration(), 0.07f + 0.01f + 0.11f, tolerance);
    EXPECT_NEAR(profile.Sample(0.13f).acceleration, -200.0f, tolerance);
}

TEST_F(TestSCurveProfileFromState, a_very_large_jerk_limit_approaches_the_trapezoid)
{
    const motion_profiles::TrapezoidalProfileFromState<float> trapezoid{ 0.0f, 10.0f, 1.5f, { 20.0f, 400.0f, 250.0f } };
    const SCurveProfileFromState<float> profile{ 0.0f, 10.0f, 0.0f, 1.5f, { 20.0f, 400.0f, 250.0f, 1e7f } };

    EXPECT_NEAR(trapezoid.Duration(), 0.12125f, 1e-5f);
    EXPECT_NEAR(profile.Duration(), trapezoid.Duration(), 1e-4f);
}

TEST_F(TestSCurveProfileFromState, beyond_the_end_it_holds_the_goal_at_rest)
{
    const SCurveProfileFromState<float> profile{ 0.0f, 0.0f, 0.0f, -1.5f, limits };

    const auto held{ profile.Sample(profile.Duration() + 1.0f) };

    EXPECT_NEAR(held.position, -1.5f, tolerance);
    EXPECT_NEAR(held.velocity, 0.0f, tolerance);
    EXPECT_NEAR(held.acceleration, 0.0f, tolerance);
}

TEST_F(TestSCurveProfileFromState, at_the_goal_and_at_rest_the_plan_is_empty)
{
    const SCurveProfileFromState<float> profile{ 1.0f, 0.0f, 0.0f, 1.0f, limits };

    EXPECT_NEAR(profile.Duration(), 0.0f, tolerance);
    EXPECT_NEAR(profile.Sample(0.0f).position, 1.0f, tolerance);
}
