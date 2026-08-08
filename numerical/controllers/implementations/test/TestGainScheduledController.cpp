#include "numerical/controllers/implementations/GainScheduledController.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>

namespace
{
    using Point1 = controllers::SchedulePoint<float, 1>;
    using Point2 = controllers::SchedulePoint<float, 2>;

    class TestGainScheduledController : public ::testing::Test
    {
    protected:
        std::array<Point1, 3> table{ { { 0.0f, { 1.0f } },
            { 0.5f, { 2.0f } },
            { 1.0f, { 4.0f } } } };
        controllers::GainScheduledController<float, 3, 1> scheduler{ table };
    };

    class TestGainScheduledControllerTwoPoint : public ::testing::Test
    {
    protected:
        std::array<Point1, 2> table{ { { 0.0f, { 0.0f } },
            { 1.0f, { 1.0f } } } };
        controllers::GainScheduledController<float, 2, 1> scheduler{ table };
    };
}

TEST_F(TestGainScheduledController, active_gains_zero_initialised_before_schedule)
{
    const auto& gains = scheduler.ActiveGains();

    EXPECT_NEAR(gains[0], 0.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_below_first_breakpoint)
{
    const auto& gains = scheduler.Schedule(-1.0f);

    EXPECT_NEAR(gains[0], 1.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_at_exact_first_breakpoint)
{
    const auto& gains = scheduler.Schedule(0.0f);

    EXPECT_NEAR(gains[0], 1.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, interpolates_midway_in_first_interval)
{
    const auto& gains = scheduler.Schedule(0.25f);

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, returns_exact_gain_at_interior_breakpoint)
{
    const auto& gains = scheduler.Schedule(0.5f);

    EXPECT_NEAR(gains[0], 2.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, interpolates_midway_in_second_interval)
{
    const auto& gains = scheduler.Schedule(0.75f);

    EXPECT_NEAR(gains[0], 3.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_at_exact_last_breakpoint)
{
    const auto& gains = scheduler.Schedule(1.0f);

    EXPECT_NEAR(gains[0], 4.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_above_last_breakpoint)
{
    const auto& gains = scheduler.Schedule(2.0f);

    EXPECT_NEAR(gains[0], 4.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, selects_second_interval_just_above_interior_breakpoint)
{
    const auto& gains = scheduler.Schedule(0.5001f);

    EXPECT_NEAR(gains[0], 2.0004f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, active_gains_persist_after_schedule)
{
    scheduler.Schedule(0.25f);
    const auto& gains = scheduler.ActiveGains();

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, sequential_schedule_calls_update_active)
{
    scheduler.Schedule(0.25f);
    scheduler.Schedule(0.75f);
    const auto& gains = scheduler.ActiveGains();

    EXPECT_NEAR(gains[0], 3.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, identical_inputs_produce_identical_outputs)
{
    const float g1{ scheduler.Schedule(0.25f)[0] };
    const float g2{ scheduler.Schedule(0.25f)[0] };

    EXPECT_FLOAT_EQ(g1, g2);
}

TEST_F(TestGainScheduledController, multi_gain_vectors_blend_componentwise)
{
    std::array<Point2, 3> t{ { { 0.0f, { 1.0f, 10.0f } },
        { 0.5f, { 2.0f, 20.0f } },
        { 1.0f, { 4.0f, 40.0f } } } };
    controllers::GainScheduledController<float, 3, 2> sched2{ t };

    const auto& gains = sched2.Schedule(0.25f);

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(gains[1], 15.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledControllerTwoPoint, interpolates_midpoint_of_minimal_table)
{
    const auto& gains = scheduler.Schedule(0.5f);

    EXPECT_NEAR(gains[0], 0.5f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledControllerTwoPoint, saturates_below_on_minimal_table)
{
    const auto& gains = scheduler.Schedule(-0.5f);

    EXPECT_NEAR(gains[0], 0.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledControllerTwoPoint, saturates_above_on_minimal_table)
{
    const auto& gains = scheduler.Schedule(1.5f);

    EXPECT_NEAR(gains[0], 1.0f, math::Tolerance<float>());
}
