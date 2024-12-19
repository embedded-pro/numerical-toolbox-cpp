#include "controllers/Pid.hpp"
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
            return {-1000, 1000};
        else
            return {T(-0.9f), T(0.9f)};
    }

    template<typename T>
    T CreateValue(float value) 
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return T(std::clamp(value, -0.9f, 0.9f));
    }

    template<typename T>
    float ToFloat(T value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return value.ToFloat();
    }
}

TYPED_TEST(TestPid, no_variation_input_results_in_no_action_control)
{
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0.1f),  // Reduced gains for stability
            CreateValue<TypeParam>(0.1f), 
            CreateValue<TypeParam>(0.1f)
        },
        std::chrono::microseconds(10000),
        CreateLimits<TypeParam>()
    );
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(0))), 0, 1e-3f);
}

TYPED_TEST(TestPid, proportional_action)
{
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0.5f),
            CreateValue<TypeParam>(0),
            CreateValue<TypeParam>(0)
        },
        std::chrono::microseconds(10000),
        CreateLimits<TypeParam>()
    );
    
    this->controller->SetPoint(CreateValue<TypeParam>(0.2f));
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(0))), 0, 1e-3f);
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(0.1f))), -0.05f, 1e-3f);
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(-0.1f))), 0.05f, 1e-3f);
}

TYPED_TEST(TestPid, integrative_action)
{
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0),
            CreateValue<TypeParam>(0.1f),
            CreateValue<TypeParam>(0)
        },
        std::chrono::microseconds(100000), // 0.1 seconds
        CreateLimits<TypeParam>()
    );
    
    this->controller->SetPoint(CreateValue<TypeParam>(0.2f));
    auto result1 = ToFloat(this->controller->Process(CreateValue<TypeParam>(0)));
    auto result2 = ToFloat(this->controller->Process(CreateValue<TypeParam>(0)));
    
    // For 0.1 second sample time, Ki=0.1, error=0.2, expect integration of 0.002 per step
    EXPECT_NEAR(result1, 0.002f, 1e-3f);
    EXPECT_NEAR(result2, 0.004f, 1e-3f);
}

TYPED_TEST(TestPid, derivative_action)
{
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0),
            CreateValue<TypeParam>(0),
            CreateValue<TypeParam>(0.1f)
        },
        std::chrono::microseconds(100000), // 0.1 seconds
        CreateLimits<TypeParam>()
    );
    
    this->controller->SetPoint(CreateValue<TypeParam>(0.2f));
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(0))), 0, 1e-3f);
    EXPECT_NEAR(ToFloat(this->controller->Process(CreateValue<TypeParam>(0.1f))), -0.1f, 1e-3f);
}

TYPED_TEST(TestPid, check_output_limits)
{
    auto limits = CreateLimits<TypeParam>();
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0.5f),
            CreateValue<TypeParam>(0.5f),
            CreateValue<TypeParam>(0.1f)
        },
        std::chrono::microseconds(100000),
        limits
    );
    
    this->controller->SetPoint(CreateValue<TypeParam>(0.8f)); // Large setpoint to test limits
    
    for (int i = 0; i < 10; ++i) 
    {
        auto output = this->controller->Process(CreateValue<TypeParam>(0));
        EXPECT_LE(ToFloat(output), ToFloat(limits.max));
        EXPECT_GE(ToFloat(output), ToFloat(limits.min));
    }
}

TYPED_TEST(TestPid, process_reaches_set_point)
{
    this->controller.emplace(
        typename controllers::Pid<TypeParam>::Tunnings{
            CreateValue<TypeParam>(0.1f),
            CreateValue<TypeParam>(0.05f),
            CreateValue<TypeParam>(0.02f)
        },
        std::chrono::microseconds(100000),
        CreateLimits<TypeParam>()
    );
    
    auto setpoint = CreateValue<TypeParam>(0.2f);
    this->controller->SetPoint(setpoint);
    EXPECT_NEAR(ToFloat(this->controller->Process(setpoint)), 0, 1e-3f);
}
