#include "controllers/SpaceVectorModulation.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    float GetTolerance()
    {
        if constexpr (std::is_same_v<T, float>)
            return 1e-3f;
        else if constexpr (std::is_same_v<T, math::Q31>)
            return 5e-2f;
        else
            return 7e-2f;
    }

    template<typename T>
    T CreateValue(float value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
        {
            if (std::abs(value) >= static_cast<float>(M_PI))
            {
                value = std::fmod(value, 2.0f * static_cast<float>(M_PI));
                if (value >= static_cast<float>(M_PI))
                    value -= 2.0f * static_cast<float>(M_PI);
                else if (value < -static_cast<float>(M_PI))
                    value += 2.0f * static_cast<float>(M_PI);
            }
            return T(0.2f * value / static_cast<float>(M_PI));
        }
    }

    template<typename T>
    controllers::RotatingFrame<T> CreateRotatingFrame(float d, float q)
    {
        if constexpr (std::is_same_v<T, float>)
            return { CreateValue<T>(d), CreateValue<T>(q) };
        else
            return { CreateValue<T>(std::clamp(d, -0.9f, 0.9f)), CreateValue<T>(std::clamp(q, -0.9f, 0.9f)) };
    }

    template<typename T>
    float GetScaledReference(float value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return value * 0.2f;
    }

    template<typename T>
    float ToFloat(T value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return value.ToFloat();
    }

    template<typename T>
    class TestSpaceVectorModulation
        : public ::testing::Test
    {
    public:
        std::optional<controllers::SpaceVectorModulation<T>> svm;

        void SetUp() override
        {
            svm.emplace(trigFunctions);
        }

    protected:
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
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSpaceVectorModulation, TestedTypes);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.0f, 0.0f);
    auto angle = CreateValue<TypeParam>(M_PI / 4);

    auto pwm = this->svm->Generate(dqVoltage, angle);
    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_1_pure_d)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto angle = CreateValue<TypeParam>(0.0f);
    auto pwm = this->svm->Generate(dqVoltage, angle);
    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(pwm.b), ToFloat(pwm.c), tolerance);
    EXPECT_NEAR(ToFloat(pwm.b) + ToFloat(pwm.a), 1.0f, tolerance);
    EXPECT_NEAR(ToFloat(pwm.c) + ToFloat(pwm.a), 1.0f, tolerance);

    EXPECT_GT(ToFloat(pwm.b), 0.5f);
    EXPECT_GT(ToFloat(pwm.c), 0.5f);
    EXPECT_LT(ToFloat(pwm.a), 0.5f);

    EXPECT_GE(ToFloat(pwm.a), 0.0f);
    EXPECT_LE(ToFloat(pwm.a), 1.0f);
    EXPECT_GE(ToFloat(pwm.b), 0.0f);
    EXPECT_LE(ToFloat(pwm.b), 1.0f);
    EXPECT_GE(ToFloat(pwm.c), 0.0f);
    EXPECT_LE(ToFloat(pwm.c), 1.0f);
}

TYPED_TEST(TestSpaceVectorModulation, overmodulation)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.9f, 0.9f);
    auto angle = CreateValue<TypeParam>(M_PI / 4);
    auto pwm = this->svm->Generate(dqVoltage, angle);

    EXPECT_GE(ToFloat(pwm.a), 0.0f);
    EXPECT_LE(ToFloat(pwm.a), 1.0f);
    EXPECT_GE(ToFloat(pwm.b), 0.0f);
    EXPECT_LE(ToFloat(pwm.b), 1.0f);
    EXPECT_GE(ToFloat(pwm.c), 0.0f);
    EXPECT_LE(ToFloat(pwm.c), 1.0f);
}

TYPED_TEST(TestSpaceVectorModulation, common_mode_injection)
{
    auto dqVoltage = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto angle = CreateValue<TypeParam>(0.0f);
    auto pwm = this->svm->Generate(dqVoltage, angle);
    float tolerance = GetTolerance<TypeParam>();

    float min_duty = std::min({ ToFloat(pwm.a), ToFloat(pwm.b), ToFloat(pwm.c) });
    float max_duty = std::max({ ToFloat(pwm.a), ToFloat(pwm.b), ToFloat(pwm.c) });

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
        auto pwm = this->svm->Generate(dq, CreateValue<TypeParam>(0.0f));
        EXPECT_GE(ToFloat(pwm.a), 0.0f);
        EXPECT_LE(ToFloat(pwm.a), 1.0f);
        EXPECT_GE(ToFloat(pwm.b), 0.0f);
        EXPECT_LE(ToFloat(pwm.b), 1.0f);
        EXPECT_GE(ToFloat(pwm.c), 0.0f);
        EXPECT_LE(ToFloat(pwm.c), 1.0f);
    }
}

TYPED_TEST(TestSpaceVectorModulation, output_linearity)
{
    auto dq_small = CreateRotatingFrame<TypeParam>(0.25f, 0.0f);
    auto dq_large = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    auto theta = CreateValue<TypeParam>(0.0f);

    auto pwm_small = this->svm->Generate(dq_small, theta);
    auto pwm_large = this->svm->Generate(dq_large, theta);

    float tolerance = GetTolerance<TypeParam>();
    float small_dev = std::abs(ToFloat(pwm_small.a) - 0.5f);
    float large_dev = std::abs(ToFloat(pwm_large.a) - 0.5f);
    EXPECT_NEAR(large_dev / small_dev, 2.0f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, zero_voltage_centering)
{
    auto zero_voltage = CreateRotatingFrame<TypeParam>(0.0f, 0.0f);
    auto pwm = this->svm->Generate(zero_voltage, CreateValue<TypeParam>(0.0f));

    float tolerance = GetTolerance<TypeParam>();
    EXPECT_NEAR(ToFloat(pwm.a), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(pwm.b), 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(pwm.c), 0.5f, tolerance);
}

TYPED_TEST(TestSpaceVectorModulation, sector_continuity)
{
    auto dq = CreateRotatingFrame<TypeParam>(0.5f, 0.0f);
    float angle = M_PI / 3.0f;

    auto pwm1 = this->svm->Generate(dq, CreateValue<TypeParam>(angle - 0.1f));
    auto pwm2 = this->svm->Generate(dq, CreateValue<TypeParam>(angle + 0.1f));

    float max_change = 0.2f;
    EXPECT_NEAR(ToFloat(pwm1.a), ToFloat(pwm2.a), max_change);
    EXPECT_NEAR(ToFloat(pwm1.b), ToFloat(pwm2.b), max_change);
    EXPECT_NEAR(ToFloat(pwm1.c), ToFloat(pwm2.c), max_change);
}
