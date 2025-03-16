#include "numerical/controllers/FieldOrientedController.hpp"
#include "numerical/controllers/test_doubles/NormalizedAngles.hpp"
#include "numerical/controllers/test_doubles/Tolerance.hpp"
#include "numerical/math/test_doubles/AdvancedFunctionsStub.hpp"
#include "numerical/math/test_doubles/TrigonometricFunctionsStub.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    typename controllers::FieldOrientedController<T>::Configuration CreateConfig()
    {
        typename controllers::FieldOrientedController<T>::Configuration config;

        config.currentTunnings.kp = T(0.5f);
        config.currentTunnings.ki = T(0.1f);
        config.currentTunnings.kd = T(0.01f);

        config.currentLimits.min = T(-0.9999f);
        config.currentLimits.max = T(0.9999f);

        return config;
    }

    template<typename T>
    class TestFieldOrientedController
        : public ::testing::Test
    {
    protected:
        math::TrigonometricFunctionsStub<T> trigFunctions;

        void SetUp() override
        {
            controller.emplace(CreateConfig<T>(), trigFunctions);
        }

        std::optional<controllers::FieldOrientedController<T>> controller;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFieldOrientedController, TestedTypes);
}

TYPED_TEST(TestFieldOrientedController, zero_current_reference)
{
    typename controllers::ThreePhase<TypeParam> phaseCurrents{
        TypeParam(0.0f),
        TypeParam(0.0f),
        TypeParam(0.0f),
    };

    this->controller->SetCurrentReference(TypeParam(0.0f));
    auto output = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(output.a), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(output.b), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(output.c), 0.5f, tolerance);
}

TYPED_TEST(TestFieldOrientedController, current_tracking)
{
    typename controllers::ThreePhase<TypeParam> phaseCurrents0{
        TypeParam(0.0f),
        TypeParam(0.0f),
        TypeParam(0.0f),
    };

    typename controllers::ThreePhase<TypeParam> phaseCurrents1{
        TypeParam(0.6f),
        TypeParam(0.0f),
        TypeParam(0.0f),
    };

    auto angle = controllers::CreateNormalizedAngle<TypeParam>(0.0f);

    this->controller->SetCurrentReference(TypeParam(0.4f));

    auto output1 = this->controller->Process(phaseCurrents0, angle);
    auto output2 = this->controller->Process(phaseCurrents1, angle);

    EXPECT_GT(math::ToFloat(output1.a) - math::ToFloat(output1.b),
        math::ToFloat(output2.a) - math::ToFloat(output2.b));

    EXPECT_GE(math::ToFloat(output1.a), 0.0f);
    EXPECT_LE(math::ToFloat(output1.a), 1.0f);
    EXPECT_GE(math::ToFloat(output2.a), 0.0f);
    EXPECT_LE(math::ToFloat(output2.a), 1.0f);
}

TYPED_TEST(TestFieldOrientedController, output_limits)
{
    typename controllers::ThreePhase<TypeParam> phaseCurrents{
        TypeParam(0.0f),
        TypeParam(0.0f),
        TypeParam(0.0f),
    };

    this->controller->SetCurrentReference(TypeParam(0.9f));
    auto output = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    EXPECT_GE(math::ToFloat(output.a), 0.0f);
    EXPECT_LE(math::ToFloat(output.a), 1.0f);
    EXPECT_GE(math::ToFloat(output.b), 0.0f);
    EXPECT_LE(math::ToFloat(output.b), 1.0f);
    EXPECT_GE(math::ToFloat(output.c), 0.0f);
    EXPECT_LE(math::ToFloat(output.c), 1.0f);
}

TYPED_TEST(TestFieldOrientedController, reset_behavior)
{
    typename controllers::ThreePhase<TypeParam> phaseCurrents{
        TypeParam(0.0f),
        TypeParam(0.0f),
        TypeParam(0.0f),
    };

    this->controller->SetCurrentReference(TypeParam(0.5f));
    auto output1 = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    this->controller->Reset();
    auto output2 = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    EXPECT_NE(math::ToFloat(output1.a), math::ToFloat(output2.a));
}

TYPED_TEST(TestFieldOrientedController, angle_dependency)
{
    typename controllers::ThreePhase<TypeParam> phaseCurrents{
        TypeParam(0.1f),
        TypeParam(0.1f),
        TypeParam(0.1f),
    };

    this->controller->SetCurrentReference(TypeParam(0.5f));
    auto output1 = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(0.0f));
    auto output2 = this->controller->Process(phaseCurrents, controllers::CreateNormalizedAngle<TypeParam>(M_PI_2));

    EXPECT_NE(math::ToFloat(output1.a), math::ToFloat(output2.a));
}
