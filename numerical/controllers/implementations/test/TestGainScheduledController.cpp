#include "numerical/controllers/implementations/GainScheduledController.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>

namespace
{
    class TestGainScheduledController
        : public ::testing::Test
    {
    public:
        using Point = controllers::SchedulePoint<float, 1>;
        std::array<Point, 3> table{ { { 0.0f, { 1.0f } },
            { 0.5f, { 2.0f } },
            { 1.0f, { 4.0f } } } };
        controllers::GainScheduledController<float, 3, 1> scheduler{ table };
    };
}

TEST_F(TestGainScheduledController, returns_exact_gain_at_breakpoint)
{
    const auto& gains = scheduler.Schedule(0.5f);

    EXPECT_NEAR(gains[0], 2.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, interpolates_midway_between_points)
{
    const auto& gains = scheduler.Schedule(0.25f);

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, interpolates_in_second_interval)
{
    const auto& gains = scheduler.Schedule(0.75f);

    EXPECT_NEAR(gains[0], 3.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_below_first_breakpoint)
{
    const auto& gains = scheduler.Schedule(-1.0f);

    EXPECT_NEAR(gains[0], 1.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, saturates_above_last_breakpoint)
{
    const auto& gains = scheduler.Schedule(2.0f);

    EXPECT_NEAR(gains[0], 4.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, selects_correct_interval)
{
    const float sJustAboveHalf{ 0.5f + 1e-4f };
    const auto& gains = scheduler.Schedule(sJustAboveHalf);

    const float expected{ 2.0f + (sJustAboveHalf - 0.5f) / (1.0f - 0.5f) * (4.0f - 2.0f) };
    EXPECT_NEAR(gains[0], expected, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, multi_gain_vectors_blend_componentwise)
{
    using Point2 = controllers::SchedulePoint<float, 2>;
    std::array<Point2, 3> table2{ { { 0.0f, { 1.0f, 10.0f } },
        { 0.5f, { 2.0f, 20.0f } },
        { 1.0f, { 4.0f, 40.0f } } } };
    controllers::GainScheduledController<float, 3, 2> scheduler2{ table2 };

    const auto& gains = scheduler2.Schedule(0.25f);

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(gains[1], 15.0f, math::Tolerance<float>());
}

TEST_F(TestGainScheduledController, active_gains_persist_between_calls)
{
    scheduler.Schedule(0.25f);
    const auto& gains = scheduler.ActiveGains();

    EXPECT_NEAR(gains[0], 1.5f, math::Tolerance<float>());
}
