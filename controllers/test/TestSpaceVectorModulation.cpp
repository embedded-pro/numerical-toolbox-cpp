#include "controllers/SpaceVectorModulation.hpp"
#include "controllers/test_doubles/NormalizedAngles.hpp"
#include "controllers/test_doubles/Tolerance.hpp"
#include "math/test_doubles/TrigonometricFunctionsStub.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    controllers::RotatingFrame<T> CreateRotatingFrame(float d, float q)
    {
        if constexpr (std::is_same_v<T, float>)
            return { T(d), T(q) };
        else
            return { T(std::clamp(d, -0.9f, 0.9f)), T(std::clamp(q, -0.9f, 0.9f)) };
    }

    template<typename T>
    class TestSpaceVectorModulation
        : public ::testing::Test
    {
    public:
        std::optional<controllers::SpaceVectorModulation<T>> spaceVectorModulation;
        dsp::TrigonometricFunctionsStub<T> trigFunctions;

        void SetUp() override
        {
            spaceVectorModulation.emplace(trigFunctions);
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSpaceVectorModulation, TestedTypes);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.0f, 0.0f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 4);

    auto pwm = this->spaceVectorModulation->Generate(dqVoltage, angle);
    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_1_pure_d)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(0.0f);
    auto pwm = this->spaceVectorModulation->Generate(dqVoltage, angle);
    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(pwm.b), math::ToFloat(pwm.c), tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.b) + math::ToFloat(pwm.a), 1.0f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.c) + math::ToFloat(pwm.a), 1.0f, tolerance);

    EXPECT_GT(math::ToFloat(pwm.b), 0.5f);
    EXPECT_GT(math::ToFloat(pwm.c), 0.5f);
    EXPECT_LT(math::ToFloat(pwm.a), 0.5f);

    EXPECT_GE(math::ToFloat(pwm.a), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.a), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.b), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.b), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.c), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.c), 1.0f);
}

TYPED_TEST(TestSpaceVectorModulation, overmodulation)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.9f, 0.9f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 4);
    auto pwm = this->spaceVectorModulation->Generate(dqVoltage, angle);

    EXPECT_GE(math::ToFloat(pwm.a), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.a), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.b), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.b), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.c), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.c), 1.0f);
}

TYPED_TEST(TestSpaceVectorModulation, common_mode_injection)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(0.0f);
    auto pwm = this->spaceVectorModulation->Generate(dqVoltage, angle);
    float tolerance = controllers::GetTolerance<TypeParam>();

    float min_duty = std::min({ math::ToFloat(pwm.a), math::ToFloat(pwm.b), math::ToFloat(pwm.c) });
    float max_duty = std::max({ math::ToFloat(pwm.a), math::ToFloat(pwm.b), math::ToFloat(pwm.c) });

    EXPECT_NEAR(min_duty + max_duty, 1.0f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, duty_cycle_bounds)
{
    auto test_points = {
        CreateRotatingFrame<TypeParam>(0.5f, 0.0f),
        CreateRotatingFrame<TypeParam>(0.0f, 0.5f),
        CreateRotatingFrame<TypeParam>(0.35f, 0.35f)
    };

    for (const auto& dq : test_points)
    {
        auto pwm = this->spaceVectorModulation->Generate(dq, controllers::CreateNormalizedAngle<TypeParam>(0.0f));
        EXPECT_GE(math::ToFloat(pwm.a), 0.0f);
        EXPECT_LE(math::ToFloat(pwm.a), 1.0f);
        EXPECT_GE(math::ToFloat(pwm.b), 0.0f);
        EXPECT_LE(math::ToFloat(pwm.b), 1.0f);
        EXPECT_GE(math::ToFloat(pwm.c), 0.0f);
        EXPECT_LE(math::ToFloat(pwm.c), 1.0f);
    }
}

TYPED_TEST(TestSpaceVectorModulation, output_linearity)
{
    auto dq_small = CreateRotatingFrame<TypeParam>(0.25f, 0.0f);
    auto dq_large = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto theta = controllers::CreateNormalizedAngle<TypeParam>(0.0f);

    auto pwm_small = this->spaceVectorModulation->Generate(dq_small, theta);
    auto pwm_large = this->spaceVectorModulation->Generate(dq_large, theta);

    float tolerance = controllers::GetTolerance<TypeParam>();
    float small_dev = std::abs(math::ToFloat(pwm_small.a) - 0.5f);
    float large_dev = std::abs(math::ToFloat(pwm_large.a) - 0.5f);
    EXPECT_NEAR(large_dev / small_dev, 2.0f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage_centering)
{
    auto zero_voltage = CreateRotatingFrame<TypeParam>(0.0f, 0.0f);
    auto pwm = this->spaceVectorModulation->Generate(zero_voltage, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    float tolerance = controllers::GetTolerance<TypeParam>();
    EXPECT_NEAR(math::ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_continuity)
{
    auto dq = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    float angle = M_PI / 3.0f;

    auto pwm1 = this->spaceVectorModulation->Generate(dq, controllers::CreateNormalizedAngle<TypeParam>(angle - 0.1f));
    auto pwm2 = this->spaceVectorModulation->Generate(dq, controllers::CreateNormalizedAngle<TypeParam>(angle + 0.1f));

    float max_change = 0.2f;
    EXPECT_NEAR(math::ToFloat(pwm1.a), math::ToFloat(pwm2.a), max_change);
    EXPECT_NEAR(math::ToFloat(pwm1.b), math::ToFloat(pwm2.b), max_change);
    EXPECT_NEAR(math::ToFloat(pwm1.c), math::ToFloat(pwm2.c), max_change);
}
