#include "controllers/Pid.hpp"
#include "gtest/gtest.h"

TEST(Pid, no_variation_input_results_in_no_action_control)
{
    controllers::Pid<math::Q31> controller({ 1, 1, 1 }, std::chrono::microseconds(10000), { -1000, 1000 });

    EXPECT_EQ(controller.Process(0), 0);
}

TEST(Pid, proportional_action)
{
    controllers::Pid<math::Q31> controller({ 1, 0, 0 }, std::chrono::microseconds(10000), { -1000, 1000 });

    controller.SetPoint(10);
    EXPECT_EQ(controller.Process(0), 0);
    EXPECT_EQ(controller.Process(5), 5);
    EXPECT_EQ(controller.Process(-5), 15);
}

TEST(Pid, proportional_action_with_negative_set_point)
{
    controllers::Pid<math::Q31> controller({ 1, 0, 0 }, std::chrono::microseconds(10000), { -1000, 1000 });

    controller.SetPoint(-10);
    EXPECT_EQ(controller.Process(0), -10);
    EXPECT_EQ(controller.Process(5), -15);
    EXPECT_EQ(controller.Process(-5), -5);
    EXPECT_EQ(controller.Process(-15), 5);
}

TEST(Pid, integrative_action)
{
    controllers::Pid<math::Q31> controller({ 0, 10, 0 }, std::chrono::microseconds(100000), { -1000, 1000 });

    controller.SetPoint(10);
    EXPECT_EQ(controller.Process(0), 10);
    EXPECT_EQ(controller.Process(0), 20);
}

TEST(Pid, integrative_action_with_negative_set_point)
{
    controllers::Pid<math::Q31> controller({ 0, 10, 0 }, std::chrono::microseconds(100000), { -1000, 1000 });

    controller.SetPoint(-10);
    EXPECT_EQ(controller.Process(0), -10);
    EXPECT_EQ(controller.Process(0), -20);
}

TEST(Pid, derivative_action)
{
    controllers::Pid<math::Q31> controller({ 0, 0, 0.1f }, std::chrono::microseconds(100000), { -1000, 1000 });

    controller.SetPoint(10);
    EXPECT_EQ(controller.Process(0), 0);
    EXPECT_EQ(controller.Process(0), 0);
    EXPECT_EQ(controller.Process(0), 0);

    EXPECT_EQ(controller.Process(5), -5);
    EXPECT_EQ(controller.Process(15), -10);
}

TEST(Pid, derivative_action_with_negative_set_point)
{
    controllers::Pid<math::Q31> controller({ 0, 0, 0.1f }, std::chrono::microseconds(100000), { -1000, 1000 });

    controller.SetPoint(-10);
    EXPECT_EQ(controller.Process(0), 0);
    EXPECT_EQ(controller.Process(0), 0);
    EXPECT_EQ(controller.Process(0), 0);

    EXPECT_EQ(controller.Process(5), -5);
    EXPECT_EQ(controller.Process(-5), 10);
    EXPECT_EQ(controller.Process(-15), 10);
}

TEST(Pid, process_reaches_set_point_results_in_no_action)
{
    controllers::Pid<math::Q31> controller({ 10, 5, 2 }, std::chrono::microseconds(100000), { -1000, 1000 });

    controller.SetPoint(10);
    EXPECT_EQ(controller.Process(10), 0);
}

TEST(Pid, check_output_limits)
{
    controllers::Pid<math::Q31> controller({ 100, 200, 40 }, std::chrono::microseconds(100000), { 0, 100 });

    controller.SetPoint(10);
    auto controlAction = controller.Process(10);

    EXPECT_LE(controlAction, 100);
    EXPECT_GE(controlAction, 0);
}
