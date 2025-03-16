#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/test_doubles/NormalizedAngles.hpp"
#include "numerical/controllers/test_doubles/Tolerance.hpp"
#include "numerical/math/test_doubles/TrigonometricFunctionsStub.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    controllers::TwoPhase<T> CreateTwoPhaseFrame(float d, float q)
    {
        return { T(d), T(q) };
    }

    template<typename T>
    class TestSpaceVectorModulation
        : public ::testing::Test
    {
    public:
        std::optional<controllers::SpaceVectorModulation<T>> spaceVectorModulation;

        void SetUp() override
        {
            spaceVectorModulation.emplace();
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSpaceVectorModulation, TestedTypes);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage)
{
    auto twoPhaseVoltage = CreateTwoPhaseFrame<TypeParam>(0.0f, 0.0f);

    auto pwm = this->spaceVectorModulation->Generate(twoPhaseVoltage);
    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_1_pure_d)
{
    auto twoPhaseVoltage = CreateTwoPhaseFrame<TypeParam>(0.5f, 0.0f);
    auto pwm = this->spaceVectorModulation->Generate(twoPhaseVoltage);
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
    auto twoPhaseVoltage = CreateTwoPhaseFrame<TypeParam>(0.5f, 0.5f);
    auto pwm = this->spaceVectorModulation->Generate(twoPhaseVoltage);

    EXPECT_GE(math::ToFloat(pwm.a), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.a), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.b), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.b), 1.0f);
    EXPECT_GE(math::ToFloat(pwm.c), 0.0f);
    EXPECT_LE(math::ToFloat(pwm.c), 1.0f);
}

TYPED_TEST(TestSpaceVectorModulation, common_mode_injection)
{
    auto twoPhaseVoltage = CreateTwoPhaseFrame<TypeParam>(0.5f, 0.0f);
    auto pwm = this->spaceVectorModulation->Generate(twoPhaseVoltage);
    float tolerance = controllers::GetTolerance<TypeParam>();

    float min_duty = std::min({ math::ToFloat(pwm.a), math::ToFloat(pwm.b), math::ToFloat(pwm.c) });
    float max_duty = std::max({ math::ToFloat(pwm.a), math::ToFloat(pwm.b), math::ToFloat(pwm.c) });

    EXPECT_NEAR(min_duty + max_duty, 1.0f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, duty_cycle_bounds)
{
    auto test_points = {
        CreateTwoPhaseFrame<TypeParam>(0.5f, 0.0f),
        CreateTwoPhaseFrame<TypeParam>(0.0f, 0.5f),
        CreateTwoPhaseFrame<TypeParam>(0.35f, 0.35f)
    };

    for (const auto& dq : test_points)
    {
        auto pwm = this->spaceVectorModulation->Generate(dq);
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
    auto dq_small = CreateTwoPhaseFrame<TypeParam>(0.05f, 0.0f);
    auto dq_large = CreateTwoPhaseFrame<TypeParam>(0.1f, 0.0f);

    auto pwm_small = this->spaceVectorModulation->Generate(dq_small);
    auto pwm_large = this->spaceVectorModulation->Generate(dq_large);

    float tolerance = controllers::GetTolerance<TypeParam>();
    float small_dev = std::abs(math::ToFloat(pwm_small.a) - 0.5f);
    float large_dev = std::abs(math::ToFloat(pwm_large.a) - 0.5f);
    EXPECT_NEAR(large_dev / small_dev, 2.0f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage_centering)
{
    auto zero_voltage = CreateTwoPhaseFrame<TypeParam>(0.0f, 0.0f);
    auto pwm = this->spaceVectorModulation->Generate(zero_voltage);

    float tolerance = controllers::GetTolerance<TypeParam>();
    EXPECT_NEAR(math::ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_continuity)
{
    auto dq = CreateTwoPhaseFrame<TypeParam>(0.5f, 0.0f);

    auto pwm1 = this->spaceVectorModulation->Generate(dq);
    auto pwm2 = this->spaceVectorModulation->Generate(dq);

    float max_change = 0.2f;
    EXPECT_NEAR(math::ToFloat(pwm1.a), math::ToFloat(pwm2.a), max_change);
    EXPECT_NEAR(math::ToFloat(pwm1.b), math::ToFloat(pwm2.b), max_change);
    EXPECT_NEAR(math::ToFloat(pwm1.c), math::ToFloat(pwm2.c), max_change);
}
