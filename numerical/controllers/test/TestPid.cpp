#include "numerical/controllers/Pid.hpp"
#include "numerical/controllers/test_doubles/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    template<typename T>
    class TestPid
        : public ::testing::Test
    {
    public:
        std::optional<controllers::Pid<T>> controller;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestPid, TestedTypes);

    template<typename T>
    typename controllers::Pid<T>::Limits CreateLimits()
    {
        if constexpr (std::is_same_v<T, float>)
            return { -1000, 1000 };
        else
            return { T(-0.9f), T(0.9f) };
    }
}

TYPED_TEST(TestPid, no_variation_input_results_in_no_action_control)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.1f),
            TypeParam(0.1f),
            TypeParam(0.1f) },
        CreateLimits<TypeParam>());
    this->controller->SetPoint(TypeParam(0.0f));
    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.0f))), 0, tolerance);
}

TYPED_TEST(TestPid, proportional_action)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.5f),
            TypeParam(0.0f),
            TypeParam(0.0f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.0f))), 0.1f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.1f))), -0.05f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(-0.1f))), 0.2f, tolerance);
}

TYPED_TEST(TestPid, integrative_action)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.0f),
            TypeParam(0.1f),
            TypeParam(0.0f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.0f))), 0.02f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.0f))), 0.02f, tolerance);
}

TYPED_TEST(TestPid, derivative_action)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.0f),
            TypeParam(0.0f),
            TypeParam(0.1f) },
        CreateLimits<TypeParam>());

    this->controller->SetPoint(TypeParam(0.2f));

    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.0f))), 0.02f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->controller->Process(TypeParam(0.1f))), -0.03f, tolerance);
}

TYPED_TEST(TestPid, check_output_limits)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    auto limits = CreateLimits<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.05f),
            TypeParam(0.05f),
            TypeParam(0.01f) },
        limits);

    this->controller->SetPoint(TypeParam(0.8f));

    for (int i = 0; i < 10; ++i)
    {
        auto output = this->controller->Process(TypeParam(0.0f));
        EXPECT_LE(math::ToFloat(output), math::ToFloat(limits.max));
        EXPECT_GE(math::ToFloat(output), math::ToFloat(limits.min));
    }
}

TYPED_TEST(TestPid, process_reaches_set_point)
{
    float tolerance = controllers::GetTolerance<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            TypeParam(0.1f),
            TypeParam(0.05f),
            TypeParam(0.02f) },
        CreateLimits<TypeParam>());

    auto setpoint = TypeParam(0.2f);
    this->controller->SetPoint(setpoint);
    EXPECT_NEAR(math::ToFloat(this->controller->Process(setpoint)), 0, tolerance);
}
