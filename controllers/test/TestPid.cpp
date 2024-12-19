#include "controllers/Pid.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestPid
        : public ::testing::Test
    {
    public:
        std::optional<controllers::Pid<float>> controller;
    };
}

TEST_F(TestPid, no_variation_input_results_in_no_action_control)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 1, 1, 1 }, std::chrono::microseconds(10000), controllers::Pid<float>::Limits{ -1000, 1000 });
    EXPECT_NEAR(controller->Process(0), 0, 1e-6f);
}

TEST_F(TestPid, proportional_action)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 1, 0, 0 }, std::chrono::microseconds(10000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(10);

    EXPECT_NEAR(controller->Process(0), 0, 1e-6f);
    EXPECT_NEAR(controller->Process(5), -5, 1e-6f);
    EXPECT_NEAR(controller->Process(-5), 5, 1e-6f);
}

TEST_F(TestPid, proportional_action_with_negative_set_point)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 1, 0, 0 }, std::chrono::microseconds(10000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(-10);

    EXPECT_NEAR(controller->Process(0), 0, 1e-6f);
    EXPECT_NEAR(controller->Process(5), -5, 1e-6f);
    EXPECT_NEAR(controller->Process(-5), 5, 1e-6f);
    EXPECT_NEAR(controller->Process(-15), 15, 1e-6f);
}

TEST_F(TestPid, integrative_action)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 0, 10, 0 }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(10);
    EXPECT_EQ(controller->Process(0), 10);
    EXPECT_EQ(controller->Process(0), 20);
}

TEST_F(TestPid, integrative_action_with_negative_set_point)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 0, 10, 0 }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(-10);
    EXPECT_EQ(controller->Process(0), -10);
    EXPECT_EQ(controller->Process(0), -20);
}

TEST_F(TestPid, derivative_action)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 0, 0, 0.1f }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(10);
    EXPECT_EQ(controller->Process(0), 0);
    EXPECT_EQ(controller->Process(0), 0);
    EXPECT_EQ(controller->Process(0), 0);

    EXPECT_EQ(controller->Process(5), -5);
    EXPECT_EQ(controller->Process(15), -10);
}

TEST_F(TestPid, derivative_action_with_negative_set_point)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 0, 0, 0.1f }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(-10);
    EXPECT_EQ(controller->Process(0), 0);
    EXPECT_EQ(controller->Process(0), 0);
    EXPECT_EQ(controller->Process(0), 0);

    EXPECT_EQ(controller->Process(5), -5);
    EXPECT_EQ(controller->Process(-5), 10);
    EXPECT_EQ(controller->Process(-15), 10);
}

TEST_F(TestPid, process_reaches_set_point_results_in_no_action)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 10, 5, 2 }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ -1000, 1000 });

    controller->SetPoint(10);
    EXPECT_EQ(controller->Process(10), 0);
}

TEST_F(TestPid, check_output_limits)
{
    controller.emplace(controllers::Pid<float>::Tunnings{ 100, 200, 40 }, std::chrono::microseconds(100000), controllers::Pid<float>::Limits{ 0, 100 });

    controller->SetPoint(10);
    auto controlAction = controller->Process(10);

    EXPECT_LE(controlAction, 100);
    EXPECT_GE(controlAction, 0);
}
