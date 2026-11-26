#include "numerical/controllers/implementations/PidContinuous.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cstddef>
#include <limits>

namespace
{
    class TestPidContinuous
        : public ::testing::Test
    {
    public:
        using Gains = controllers::PidContinuous<float>::Gains;

        const float tolerance{ math::Tolerance<float>() };

        controllers::PidContinuous<float> pid{ -10.0f, 10.0f };
    };
}

TEST_F(TestPidContinuous, proportional_only_follows_the_error)
{
    const Gains gains{ 2.0f, 0.0f, 0.0f };

    EXPECT_NEAR(pid.Update(1.0f, 0.0f, gains, 0.01f), 2.0f, tolerance);
    EXPECT_NEAR(pid.Update(1.0f, 0.0f, gains, 0.01f), 2.0f, tolerance);
}

TEST_F(TestPidContinuous, integral_matches_the_continuous_integral)
{
    const Gains gains{ 0.0f, 3.0f, 0.0f };

    for (int step = 0; step != 100; ++step)
        pid.Update(0.5f, 0.0f, gains, 0.01f);

    EXPECT_NEAR(pid.Output(), 1.5f, tolerance);
}

TEST_F(TestPidContinuous, integral_uses_each_measured_interval)
{
    const Gains gains{ 0.0f, 1.0f, 0.0f };

    pid.Update(1.0f, 0.0f, gains, 0.01f);
    pid.Update(1.0f, 0.0f, gains, 0.03f);

    EXPECT_NEAR(pid.Update(1.0f, 0.0f, gains, 0.06f), 0.1f, tolerance);
}

TEST_F(TestPidContinuous, derivative_of_a_ramp_is_constant_from_the_second_step)
{
    const Gains gains{ 0.0f, 0.0f, 0.2f };

    EXPECT_NEAR(pid.Update(0.1f, 0.0f, gains, 0.01f), 0.0f, tolerance);
    EXPECT_NEAR(pid.Update(0.2f, 0.0f, gains, 0.01f), 2.0f, tolerance);
    EXPECT_NEAR(pid.Update(0.3f, 0.0f, gains, 0.01f), 2.0f, tolerance);
}

TEST_F(TestPidContinuous, a_setpoint_step_gives_no_derivative_kick)
{
    EXPECT_NEAR(pid.Update(1.0f, 0.0f, Gains{ 0.0f, 0.0f, 1.0f }, 0.01f), 0.0f, tolerance);
}

TEST_F(TestPidContinuous, the_clamped_output_does_not_wind_up)
{
    const Gains gains{ 0.0f, 100.0f, 0.0f };

    for (int step = 0; step != 200; ++step)
        pid.Update(1.0f, 0.0f, gains, 0.01f);
    EXPECT_NEAR(pid.Output(), 10.0f, tolerance);
    EXPECT_LT(pid.Update(-1.0f, 0.0f, gains, 0.01f), 10.0f);

    for (int step = 0; step != 400; ++step)
        pid.Update(-1.0f, 0.0f, gains, 0.01f);
    EXPECT_NEAR(pid.Output(), -10.0f, tolerance);
    EXPECT_GT(pid.Update(1.0f, 0.0f, gains, 0.01f), -10.0f);
}

TEST_F(TestPidContinuous, bad_input_is_ignored_and_leaves_the_state_unchanged)
{
    const Gains gains{ 1.0f, 1.0f, 1.0f };
    controllers::PidContinuous<float> fresh{ -10.0f, 10.0f };
    const float output{ pid.Update(1.0f, 0.0f, gains, 0.01f) };
    fresh.Update(1.0f, 0.0f, gains, 0.01f);
    const float nan{ std::numeric_limits<float>::quiet_NaN() };
    const float infinity{ std::numeric_limits<float>::infinity() };

    EXPECT_NEAR(pid.Update(5.0f, 0.0f, gains, 0.0f), output, tolerance);
    EXPECT_NEAR(pid.Update(5.0f, 0.0f, gains, -1.0f), output, tolerance);
    EXPECT_NEAR(pid.Update(5.0f, 0.0f, gains, nan), output, tolerance);
    EXPECT_NEAR(pid.Update(5.0f, 0.0f, gains, infinity), output, tolerance);
    EXPECT_NEAR(pid.Update(nan, 0.0f, gains, 0.01f), output, tolerance);
    EXPECT_NEAR(pid.Update(1.0f, infinity, gains, 0.01f), output, tolerance);
    EXPECT_NEAR(pid.Update(1.0f, 0.0f, Gains{ nan, 1.0f, 1.0f }, 0.01f), output, tolerance);

    EXPECT_NEAR(pid.Update(2.0f, 0.0f, gains, 0.01f), fresh.Update(2.0f, 0.0f, gains, 0.01f), tolerance);
}

TEST_F(TestPidContinuous, the_derivative_uses_each_measured_interval)
{
    const Gains gains{ 0.0f, 0.0f, 0.1f };

    pid.Update(0.0f, 0.0f, gains, 0.01f);
    EXPECT_NEAR(pid.Update(0.2f, 0.0f, gains, 0.02f), 1.0f, tolerance);
    EXPECT_NEAR(pid.Update(0.5f, 0.0f, gains, 0.05f), 0.6f, tolerance);
}

TEST_F(TestPidContinuous, reset_clears_the_state)
{
    const Gains gains{ 1.0f, 1.0f, 1.0f };
    pid.Update(1.0f, 0.0f, gains, 0.01f);
    pid.Update(3.0f, 0.0f, gains, 0.01f);

    pid.Reset();

    EXPECT_NEAR(pid.Output(), 0.0f, tolerance);
    EXPECT_NEAR(pid.Update(1.0f, 0.0f, gains, 0.01f), 1.01f, tolerance);
}

TEST_F(TestPidContinuous, matches_the_per_sample_pid_at_a_fixed_step_except_for_the_first_step_kick)
{
    const Gains gains{ 1.5f, 4.0f, 0.02f };
    const float interval{ 0.01f };
    controllers::PidIncrementalSynchronous<float> reference{ { gains.kp, gains.ki * interval, gains.kd / interval }, { -10.0f, 10.0f } };
    reference.SetPoint(0.0f);
    const std::array<float, 6> measured{ 0.2f, 0.3f, 0.5f, 0.4f, 0.1f, -0.2f };

    const float kick{ gains.kd / interval * -measured[0] };
    EXPECT_NEAR(pid.Update(0.0f, measured[0], gains, interval) + kick, reference.Process(measured[0]), tolerance);

    for (std::size_t i = 1; i != measured.size(); ++i)
        EXPECT_NEAR(pid.Update(0.0f, measured[i], gains, interval), reference.Process(measured[i]), tolerance);
}
