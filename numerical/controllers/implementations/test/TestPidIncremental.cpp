#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/controllers/implementations/test_doubles/PidDriverMock.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    using PidTypes = ::testing::Types<float, math::Q15, math::Q31>;

    template<typename T>
    class TestPidIncrementalSynchronous : public ::testing::Test
    {
    public:
        controllers::PidIncrementalSynchronous<T> MakeController(
            controllers::PidTunings<T> tunings,
            controllers::PidLimits<T> limits)
        {
            return controllers::PidIncrementalSynchronous<T>{ tunings, limits };
        }

        controllers::PidTunings<T> PGains(float kp)
        {
            return { T(kp), T(0.0f), T(0.0f) };
        }

        controllers::PidTunings<T> IGains(float ki)
        {
            return { T(0.0f), T(ki), T(0.0f) };
        }

        controllers::PidTunings<T> DGains(float kd)
        {
            return { T(0.0f), T(0.0f), T(kd) };
        }

        controllers::PidTunings<T> PIDGains(float kp, float ki, float kd)
        {
            return { T(kp), T(ki), T(kd) };
        }

        controllers::PidLimits<T> WideLimit()
        {
            return { T(-0.9f), T(0.9f) };
        }

        controllers::PidLimits<T> NarrowLimit()
        {
            return { T(-0.1f), T(0.1f) };
        }
    };

    TYPED_TEST_SUITE(TestPidIncrementalSynchronous, PidTypes);

    class TestPidIncrementalAsynchronous : public ::testing::Test
    {
    public:
        ::testing::StrictMock<controllers::MockPidDriver<float>> driver;
        std::optional<controllers::PidIncrementalAsynchronous<float>> controller;
        std::chrono::system_clock::duration sampleTime;
        infra::Function<void(float)> readCallback;

        TestPidIncrementalAsynchronous()
            : sampleTime{ std::chrono::milliseconds(100) }
        {}

        void CreateController(controllers::PidTunings<float> tunings, controllers::PidLimits<float> limits)
        {
            using ::testing::_;
            using ::testing::SaveArg;

            EXPECT_CALL(driver, Read(_))
                .WillOnce(SaveArg<0>(&readCallback));
            EXPECT_CALL(driver, Start(_));

            controller.emplace(driver, sampleTime, tunings, limits);
        }

        void TearDown() override
        {
            if (controller.has_value())
            {
                EXPECT_CALL(driver, Stop());
                controller.reset();
            }
        }

        float ProcessValue(float measuredValue)
        {
            float output{};
            EXPECT_CALL(driver, ControlAction(::testing::_))
                .WillOnce(::testing::SaveArg<0>(&output));
            readCallback(measuredValue);
            return output;
        }
    };
}

TYPED_TEST(TestPidIncrementalSynchronous, zero_error_produces_zero_increment)
{
    auto controller = this->MakeController(this->PIDGains(0.1f, 0.1f, 0.1f), this->WideLimit());
    controller.SetPoint(TypeParam(0.0f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.0f))), 0.0f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, proportional_first_step)
{
    auto controller = this->MakeController(this->PGains(0.5f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.0f))), 0.1f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, proportional_tracks_error_change)
{
    auto controller = this->MakeController(this->PGains(0.5f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    controller.Process(TypeParam(0.0f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.1f))), 0.05f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, integral_accumulates_over_two_steps)
{
    auto controller = this->MakeController(this->IGains(0.1f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.0f))), 0.02f, math::Tolerance<TypeParam>());
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.0f))), 0.04f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, integral_achieves_zero_steady_state_error)
{
    auto controller = this->MakeController(this->PIDGains(0.3f, 0.1f, 0.0f), this->WideLimit());
    controller.SetPoint(TypeParam(0.5f));
    TypeParam pv{};
    for (int i = 0; i < 200; ++i)
    {
        TypeParam output = controller.Process(pv);
        pv = TypeParam(math::ToFloat(pv) + 0.05f * (math::ToFloat(output) - math::ToFloat(pv)));
    }
    EXPECT_NEAR(math::ToFloat(pv), 0.5f, 0.05f);
}

TYPED_TEST(TestPidIncrementalSynchronous, derivative_first_step)
{
    auto controller = this->MakeController(this->DGains(0.1f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.0f))), 0.02f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, derivative_second_step_opposes_error_reduction)
{
    auto controller = this->MakeController(this->DGains(0.1f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    controller.Process(TypeParam(0.0f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.1f))), -0.01f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, output_clamped_at_max_limit)
{
    auto limits = this->NarrowLimit();
    auto controller = this->MakeController(this->PGains(0.5f), limits);
    controller.SetPoint(TypeParam(0.4f));
    EXPECT_FLOAT_EQ(math::ToFloat(controller.Process(TypeParam(0.0f))), math::ToFloat(limits.max));
}

TYPED_TEST(TestPidIncrementalSynchronous, output_clamped_at_min_limit)
{
    auto limits = this->NarrowLimit();
    auto controller = this->MakeController(this->PGains(0.5f), limits);
    controller.SetPoint(TypeParam(-0.4f));
    EXPECT_FLOAT_EQ(math::ToFloat(controller.Process(TypeParam(0.0f))), math::ToFloat(limits.min));
}

TYPED_TEST(TestPidIncrementalSynchronous, saturation_clamps_all_steps)
{
    auto limits = this->NarrowLimit();
    auto controller = this->MakeController(this->PIDGains(0.05f, 0.05f, 0.01f), limits);
    controller.SetPoint(TypeParam(0.8f));
    for (int i = 0; i < 10; ++i)
    {
        auto output = controller.Process(TypeParam(0.0f));
        EXPECT_LE(output, limits.max);
        EXPECT_GE(output, limits.min);
    }
}

TYPED_TEST(TestPidIncrementalSynchronous, anti_windup_output_recovers_after_saturation)
{
    auto limits = this->NarrowLimit();
    auto controller = this->MakeController(this->PIDGains(0.1f, 0.1f, 0.0f), limits);
    controller.SetPoint(TypeParam(0.9f));
    for (int i = 0; i < 20; ++i)
        controller.Process(TypeParam(0.0f));
    controller.SetPoint(TypeParam(-0.9f));
    auto output = controller.Process(TypeParam(0.0f));
    EXPECT_LE(output, limits.max);
    EXPECT_GE(output, limits.min);
}

TYPED_TEST(TestPidIncrementalSynchronous, no_setpoint_returns_process_variable)
{
    auto controller = this->MakeController(this->PGains(0.5f), this->WideLimit());
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.3f))), 0.3f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, set_tunings_changes_gain_immediately)
{
    auto controller = this->MakeController(this->PGains(0.5f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    controller.Process(TypeParam(0.0f));
    controller.SetTunings(this->PIDGains(0.25f, 0.0f, 0.0f));
    EXPECT_NEAR(math::ToFloat(controller.Process(TypeParam(0.2f))), 0.05f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestPidIncrementalSynchronous, reset_restores_state_equal_to_fresh_instance)
{
    auto controller = this->MakeController(this->PGains(0.5f), this->WideLimit());
    controller.SetPoint(TypeParam(0.2f));
    controller.Process(TypeParam(0.0f));
    controller.Reset();
    float afterReset = math::ToFloat(controller.Process(TypeParam(0.0f)));

    auto fresh = this->MakeController(this->PGains(0.5f), this->WideLimit());
    fresh.SetPoint(TypeParam(0.2f));
    float freshFirst = math::ToFloat(fresh.Process(TypeParam(0.0f)));

    EXPECT_FLOAT_EQ(afterReset, freshFirst);
}

TYPED_TEST(TestPidIncrementalSynchronous, two_identical_runs_produce_same_output)
{
    auto controller = this->MakeController(this->PIDGains(0.5f, 0.1f, 0.05f), this->WideLimit());
    controller.SetPoint(TypeParam(0.3f));
    float run1 = math::ToFloat(controller.Process(TypeParam(0.1f)));

    controller.Reset();
    controller.SetPoint(TypeParam(0.3f));
    float run2 = math::ToFloat(controller.Process(TypeParam(0.1f)));

    EXPECT_FLOAT_EQ(run1, run2);
}

TYPED_TEST(TestPidIncrementalSynchronous, output_within_limits_not_clamped)
{
    auto limits = this->WideLimit();
    auto controller = this->MakeController(this->PGains(0.1f), limits);
    controller.SetPoint(TypeParam(0.1f));
    auto output = controller.Process(TypeParam(0.0f));
    EXPECT_NEAR(math::ToFloat(output), 0.01f, math::Tolerance<TypeParam>());
    EXPECT_LT(output, limits.max);
    EXPECT_GT(output, limits.min);
}

TEST_F(TestPidIncrementalAsynchronous, zero_error_produces_zero_control_action)
{
    CreateController({ 0.1f, 0.1f, 0.1f }, { -0.9f, 0.9f });
    controller->SetPoint(0.0f);
    EXPECT_NEAR(ProcessValue(0.0f), 0.0f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, proportional_first_step)
{
    CreateController({ 0.5f, 0.0f, 0.0f }, { -0.9f, 0.9f });
    controller->SetPoint(0.2f);
    EXPECT_NEAR(ProcessValue(0.0f), 0.1f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, integral_accumulates_over_two_steps)
{
    CreateController({ 0.0f, 0.1f, 0.0f }, { -0.9f, 0.9f });
    controller->SetPoint(0.2f);
    EXPECT_NEAR(ProcessValue(0.0f), 0.02f, math::Tolerance<float>());
    EXPECT_NEAR(ProcessValue(0.0f), 0.04f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, derivative_second_step_opposes_error_reduction)
{
    CreateController({ 0.0f, 0.0f, 0.1f }, { -0.9f, 0.9f });
    controller->SetPoint(0.2f);
    ProcessValue(0.0f);
    EXPECT_NEAR(ProcessValue(0.1f), -0.01f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, output_clamped_at_max_limit)
{
    auto limits = controllers::PidLimits<float>{ -0.1f, 0.1f };
    CreateController({ 0.5f, 0.0f, 0.0f }, limits);
    controller->SetPoint(0.4f);
    EXPECT_FLOAT_EQ(ProcessValue(0.0f), limits.max);
}

TEST_F(TestPidIncrementalAsynchronous, output_clamped_at_min_limit)
{
    auto limits = controllers::PidLimits<float>{ -0.1f, 0.1f };
    CreateController({ 0.5f, 0.0f, 0.0f }, limits);
    controller->SetPoint(-0.4f);
    EXPECT_FLOAT_EQ(ProcessValue(0.0f), limits.min);
}

TEST_F(TestPidIncrementalAsynchronous, no_setpoint_returns_process_variable)
{
    CreateController({ 0.5f, 0.0f, 0.0f }, { -0.9f, 0.9f });
    EXPECT_NEAR(ProcessValue(0.3f), 0.3f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, reset_restores_state_to_initial)
{
    CreateController({ 0.5f, 0.0f, 0.0f }, { -0.9f, 0.9f });
    controller->SetPoint(0.2f);
    ProcessValue(0.0f);
    controller->Reset();
    EXPECT_NEAR(ProcessValue(0.0f), 0.1f, math::Tolerance<float>());
}

TEST_F(TestPidIncrementalAsynchronous, set_tunings_changes_gain_immediately)
{
    CreateController({ 0.5f, 0.0f, 0.0f }, { -0.9f, 0.9f });
    controller->SetPoint(0.2f);
    ProcessValue(0.0f);
    controller->SetTunings({ 0.25f, 0.0f, 0.0f });
    EXPECT_NEAR(ProcessValue(0.2f), 0.05f, math::Tolerance<float>());
}
