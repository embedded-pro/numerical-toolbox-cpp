#include "controllers/FieldOrientedController.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    T CreateValue(float value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return T(value * 0.2f);
    }

    template<typename T>
    float ToFloat(T value, float scale = 0.2f)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return value.ToFloat() / scale;
    }

    template<typename T>
    typename controllers::FieldOrientedController<T>::Configuration CreateConfig()
    {
        typename controllers::FieldOrientedController<T>::Configuration config;

        config.currentTunnings.kp = CreateValue<T>(0.5f);
        config.currentTunnings.ki = CreateValue<T>(0.1f);
        config.currentTunnings.kd = CreateValue<T>(0.01f);

        config.currentLimits.min = CreateValue<T>(-1.0f);
        config.currentLimits.max = CreateValue<T>(1.0f);

        config.sampleTime = std::chrono::microseconds(100000);
        return config;
    }

    template<typename T>
    class TestFieldOrientedController
        : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            controller.emplace(CreateConfig<T>(), trigFunctions, advancedFunctions);
        }

        class MockTrigonometricFunctions
            : public dsp::TrigonometricFunctions<T>
        {
        public:
            T Cosine(const T& angle) const override
            {
                return CreateValue<T>(std::cos(ToFloat(angle)));
            }

            T Sine(const T& angle) const override
            {
                return CreateValue<T>(std::sin(ToFloat(angle)));
            }

            T Arctangent(const T& value) const override
            {
                return CreateValue<T>(std::atan(ToFloat(value)));
            }

            T Phase(const T& real, const T& imag) const override
            {
                return CreateValue<T>(std::atan2(ToFloat(imag), ToFloat(real)));
            }
        } trigFunctions;

        class MockAdvancedFunctions
            : public dsp::AdvancedFunctions<T>
        {
        public:
            T Modulus(const T& real, const T& imag) const override
            {
                return CreateValue<T>(std::sqrt(std::pow(ToFloat(real), 2) + std::pow(ToFloat(imag), 2)));
            }

            T NaturalLogarithm(const T& value) const override
            {
                return CreateValue<T>(std::log(ToFloat(value)));
            }

            T SquareRoot(const T& value) const override
            {
                return CreateValue<T>(std::sqrt(ToFloat(value)));
            }
        } advancedFunctions;

        std::optional<controllers::FieldOrientedController<T>> controller;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFieldOrientedController, TestedTypes);
}

TYPED_TEST(TestFieldOrientedController, zero_current_reference)
{
    typename controllers::FieldOrientedController<TypeParam>::CurrentReferences refs{
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f)
    };

    typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements meas{
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f)
    };

    this->controller->SetCurrentReferences(refs);
    auto output = this->controller->Process(refs, meas);

    float tolerance = std::is_same_v<TypeParam, float> ? 1e-3f : 5e-2f;

    EXPECT_NEAR(ToFloat(output.a, 1.0f), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(output.b, 1.0f), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(output.c, 1.0f), 0.5f, tolerance);
}

TYPED_TEST(TestFieldOrientedController, current_tracking)
{
    typename controllers::FieldOrientedController<TypeParam>::CurrentReferences refs{
        CreateValue<TypeParam>(0.4f),
        CreateValue<TypeParam>(0.0f)
    };

    std::vector<typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements> measurements = {
        { CreateValue<TypeParam>(0.0f),
            CreateValue<TypeParam>(0.0f),
            CreateValue<TypeParam>(0.0f),
            CreateValue<TypeParam>(0.0f) },
        { CreateValue<TypeParam>(0.6f),
            CreateValue<TypeParam>(0.0f),
            CreateValue<TypeParam>(0.0f),
            CreateValue<TypeParam>(0.0f) }
    };

    this->controller->SetCurrentReferences(refs);

    auto output1 = this->controller->Process(refs, measurements[0]);
    auto output2 = this->controller->Process(refs, measurements[1]);

    EXPECT_GT(ToFloat(output1.a) - ToFloat(output1.b),
        ToFloat(output2.a) - ToFloat(output2.b));

    EXPECT_GE(ToFloat(output1.a, 1.0f), 0.0f);
    EXPECT_LE(ToFloat(output1.a, 1.0f), 1.0f);
    EXPECT_GE(ToFloat(output2.a, 1.0f), 0.0f);
    EXPECT_LE(ToFloat(output2.a, 1.0f), 1.0f);
}

TYPED_TEST(TestFieldOrientedController, output_limits)
{
    typename controllers::FieldOrientedController<TypeParam>::CurrentReferences refs{
        CreateValue<TypeParam>(2.0f),
        CreateValue<TypeParam>(2.0f)
    };

    typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements meas{
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f)
    };

    this->controller->SetCurrentReferences(refs);
    auto output = this->controller->Process(refs, meas);

    EXPECT_GE(ToFloat(output.a, 1.0f), 0.0f);
    EXPECT_LE(ToFloat(output.a, 1.0f), 1.0f);
    EXPECT_GE(ToFloat(output.b, 1.0f), 0.0f);
    EXPECT_LE(ToFloat(output.b, 1.0f), 1.0f);
    EXPECT_GE(ToFloat(output.c, 1.0f), 0.0f);
    EXPECT_LE(ToFloat(output.c, 1.0f), 1.0f);
}

TYPED_TEST(TestFieldOrientedController, reset_behavior)
{
    typename controllers::FieldOrientedController<TypeParam>::CurrentReferences refs{
        CreateValue<TypeParam>(0.5f),
        CreateValue<TypeParam>(0.5f)
    };

    typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements meas{
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f),
        CreateValue<TypeParam>(0.0f)
    };

    this->controller->SetCurrentReferences(refs);
    auto output1 = this->controller->Process(refs, meas);

    this->controller->Reset();
    auto output2 = this->controller->Process(refs, meas);

    EXPECT_NE(ToFloat(output1.a), ToFloat(output2.a));
}

TYPED_TEST(TestFieldOrientedController, angle_dependency)
{
    typename controllers::FieldOrientedController<TypeParam>::CurrentReferences refs{
        CreateValue<TypeParam>(0.5f),
        CreateValue<TypeParam>(0.0f)
    };

    typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements meas1{
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(0.0f)
    };

    typename controllers::FieldOrientedController<TypeParam>::PhaseMeasurements meas2{
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(0.1f),
        CreateValue<TypeParam>(M_PI_2)
    };

    this->controller->SetCurrentReferences(refs);
    auto output1 = this->controller->Process(refs, meas1);
    auto output2 = this->controller->Process(refs, meas2);

    EXPECT_NE(ToFloat(output1.a), ToFloat(output2.a));
}
