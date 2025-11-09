#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/controllers/implementations/test_doubles/PidDriverMock.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    template<typename T>
    class TestPidIncremental
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<controllers::MockPidDriver<T>> driver;
        std::optional<controllers::PidIncrementalAsynchronous<T>> controller;
        std::chrono::system_clock::duration sampleTime{ std::chrono::milliseconds(100) };
        infra::Function<void(T)> readCallback;

        void CreateController(controllers::PidTunings<T> tunings, controllers::PidLimits<T> limits)
        {
            using ::testing::_;
            using ::testing::SaveArg;

            EXPECT_CALL(driver, Read(_))
                .WillOnce(SaveArg<0>(&readCallback));

            controller.emplace(driver, sampleTime, tunings, limits);
        }

        T ProcessValue(T measuredValue)
        {
            T output{};

            EXPECT_CALL(driver, ControlAction(::testing::_))
                .WillOnce(::testing::SaveArg<0>(&output));

            readCallback(measuredValue);
            return output;
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestPidIncremental, TestedTypes);

    template<typename T>
    typename controllers::PidLimits<T> CreateLimits()
    {
        if constexpr (std::is_same_v<T, float>)
            return { -1000, 1000 };
        else
            return { T(-0.9f), T(0.9f) };
    }
}

TYPED_TEST(TestPidIncremental, no_variation_input_results_in_no_action_control)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.1f),
            TypeParam(0.1f),
            TypeParam(0.1f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.0f));

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.0f))), 0, tolerance);
}

TYPED_TEST(TestPidIncremental, proportional_action)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.5f),
            TypeParam(0.0f),
            TypeParam(0.0f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.0f))), 0.1f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.1f))), 0.05f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(-0.1f))), 0.15f, tolerance);
}

TYPED_TEST(TestPidIncremental, integrative_action)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.0f),
            TypeParam(0.1f),
            TypeParam(0.0f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.0f))), 0.02f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.0f))), 0.04f, tolerance);
}

TYPED_TEST(TestPidIncremental, derivative_action)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.0f),
            TypeParam(0.0f),
            TypeParam(0.1f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.0f))), 0.02f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->ProcessValue(TypeParam(0.1f))), -0.01f, tolerance);
}

TYPED_TEST(TestPidIncremental, check_output_limits)
{
    float tolerance = math::Tolerance<TypeParam>();
    auto limits = CreateLimits<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.05f),
            TypeParam(0.05f),
            TypeParam(0.01f) },
        limits);

    this->controller->SetPoint(TypeParam(0.8f));

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    for (int i = 0; i < 10; ++i)
    {
        auto output = this->ProcessValue(TypeParam(0.0f));
        EXPECT_LE(math::ToFloat(output), math::ToFloat(limits.max));
        EXPECT_GE(math::ToFloat(output), math::ToFloat(limits.min));
    }
}

TYPED_TEST(TestPidIncremental, process_reaches_set_point)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->CreateController(
        typename controllers::PidTunings<TypeParam>{
            TypeParam(0.1f),
            TypeParam(0.05f),
            TypeParam(0.02f) },
        CreateLimits<TypeParam>());

    auto setpoint = TypeParam(0.2f);
    this->controller->SetPoint(setpoint);

    EXPECT_CALL(this->driver, Start(::testing::_));
    this->controller->Enable();

    EXPECT_NEAR(math::ToFloat(this->ProcessValue(setpoint)), 0, tolerance);
}
