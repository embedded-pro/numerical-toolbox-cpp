#include "numerical/controllers/TransformsClarkePark.hpp"
#include "numerical/controllers/test_doubles/NormalizedAngles.hpp"
#include "numerical/controllers/test_doubles/Tolerance.hpp"
#include "numerical/math/test_doubles/AdvancedFunctionsStub.hpp"
#include "numerical/math/test_doubles/TrigonometricFunctionsStub.hpp"
#include <gmock/gmock.h>

namespace
{
    template<typename T>
    controllers::ThreePhase<T> CreateThreePhase(float a, float b, float c)
    {
        return { T(a), T(b), (c) };
    }

    template<typename T>
    controllers::TwoPhase<T> CreateTwoPhase(float alpha, float beta)
    {
        return { T(alpha), T(beta) };
    }

    template<typename T>
    class TestTransforms
        : public ::testing::Test
    {
    public:
        std::optional<controllers::Clarke<T>> clarke;
        std::optional<controllers::Park<T>> park;
        std::optional<controllers::ClarkePark<T>> clarkePark;
        math::AdvancedFunctionsStub<T> advancedFunctions;
        math::TrigonometricFunctionsStub<T> trigFunctions;

        void SetUp() override
        {
            clarke.emplace(advancedFunctions);
            park.emplace(trigFunctions);
            clarkePark.emplace(trigFunctions, advancedFunctions);
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestTransforms, TestedTypes);
}

TYPED_TEST(TestTransforms, clarke_balanced_system)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.25f, -0.25f);
    auto result = this->clarke->Forward(input);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.alpha), 0.5f, tolerance);
    EXPECT_NEAR(math::ToFloat(result.beta), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, clarke_zero_input)
{
    auto input = CreateThreePhase<TypeParam>(0.0f, 0.0f, 0.0f);
    auto result = this->clarke->Forward(input);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.alpha), 0.0f, tolerance);
    EXPECT_NEAR(math::ToFloat(result.beta), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, clarke_inverse_recovers_original)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.2f, -0.3f);
    auto alphabeta = this->clarke->Forward(input);
    auto result = this->clarke->Inverse(alphabeta);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.a), math::ToFloat(input.a), tolerance);
    EXPECT_NEAR(math::ToFloat(result.b), math::ToFloat(input.b), tolerance);
    EXPECT_NEAR(math::ToFloat(result.c), math::ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, park_zero_angle)
{
    auto input = CreateTwoPhase<TypeParam>(0.3f, 0.0f);
    auto result = this->park->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(0.0f));

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.d), math::ToFloat(input.alpha), tolerance);
    EXPECT_NEAR(math::ToFloat(result.q), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, park_ninety_degrees)
{
    auto input = CreateTwoPhase<TypeParam>(0.1f, 0.0f);
    auto result = this->park->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(M_PI_2));

    float tolerance = controllers::GetTolerance<TypeParam>();

    printf("d = %f, expected: %f\n", math::ToFloat(result.d), 0.0f);
    printf("q = %f, expected: %f\n", math::ToFloat(result.q), -math::ToFloat(input.alpha));

    EXPECT_NEAR(math::ToFloat(result.d), 0.0f, tolerance);
    EXPECT_NEAR(math::ToFloat(result.q), -math::ToFloat(input.alpha), tolerance);
}

TYPED_TEST(TestTransforms, park_inverse_recovers_original)
{
    auto input = CreateTwoPhase<TypeParam>(0.5f, 0.3f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 4);
    auto dq = this->park->Forward(input, angle);
    auto result = this->park->Inverse(dq, angle);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.alpha), math::ToFloat(input.alpha), tolerance);
    EXPECT_NEAR(math::ToFloat(result.beta), math::ToFloat(input.beta), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_full_transform)
{
    auto input = CreateThreePhase<TypeParam>(0.4f, -0.2f, -0.2f);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 6);
    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.a), math::ToFloat(input.a), tolerance);
    EXPECT_NEAR(math::ToFloat(result.b), math::ToFloat(input.b), tolerance);
    EXPECT_NEAR(math::ToFloat(result.c), math::ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_multiple_angles)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.25f, -0.25f);
    std::vector<float> angles = { 0.0f, M_PI_4, M_PI_2, 3 * M_PI_4, M_PI };

    float tolerance = controllers::GetTolerance<TypeParam>();

    for (const auto& angle : angles)
    {
        auto dq = this->clarkePark->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(angle));
        auto result = this->clarkePark->Inverse(dq, controllers::CreateNormalizedAngle<TypeParam>(angle));

        EXPECT_NEAR(math::ToFloat(result.a), math::ToFloat(input.a), tolerance);
        EXPECT_NEAR(math::ToFloat(result.b), math::ToFloat(input.b), tolerance);
        EXPECT_NEAR(math::ToFloat(result.c), math::ToFloat(input.c), tolerance);
    }
}

TYPED_TEST(TestTransforms, clarke_unbalanced_system)
{
    auto input = CreateThreePhase<TypeParam>(0.4, -0.05, -0.2);
    auto result = this->clarke->Forward(input);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.alpha), 0.35f, tolerance);
    EXPECT_NEAR(math::ToFloat(result.beta), 0.0866f, tolerance);
}

TYPED_TEST(TestTransforms, park_negative_angles)
{
    auto input = CreateTwoPhase<TypeParam>(0.3f, 0.2f);
    auto result = this->park->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(-M_PI_4));

    float tolerance = controllers::GetTolerance<TypeParam>();

    float cos45 = std::cos(-M_PI_4);
    float sin45 = std::sin(-M_PI_4);

    EXPECT_NEAR(math::ToFloat(result.d), 0.3f * cos45 + 0.2f * sin45, tolerance);
    EXPECT_NEAR(math::ToFloat(result.q), -0.3f * sin45 + 0.2f * cos45, tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_near_limits)
{
    float max_val = 0.5f;
    auto input = CreateThreePhase<TypeParam>(max_val, -max_val / 2, -max_val / 2);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 3);

    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.a), math::ToFloat(input.a), tolerance);
    EXPECT_NEAR(math::ToFloat(result.b), math::ToFloat(input.b), tolerance);
    EXPECT_NEAR(math::ToFloat(result.c), math::ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, clarke_dc_offset)
{
    auto input = CreateThreePhase<TypeParam>(0.6f, 0.1f, 0.1f);
    auto result = this->clarke->Forward(input);

    float tolerance = controllers::GetTolerance<TypeParam>();

    EXPECT_NEAR(math::ToFloat(result.alpha), 2.0f / 3.0f * (0.6f - 0.1f), tolerance);
    EXPECT_NEAR(math::ToFloat(result.beta), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, park_harmonic_angle)
{
    auto input = CreateTwoPhase<TypeParam>(0.2f, 0.3f);
    float base_rads = M_PI / 6;
    float harmonic_rads = std::fmod(base_rads + 2 * M_PI, 2 * M_PI);
    float tolerance = controllers::GetTolerance<TypeParam>();

    auto result1 = this->park->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(base_rads));
    auto result2 = this->park->Forward(input, controllers::CreateNormalizedAngle<TypeParam>(harmonic_rads));

    EXPECT_NEAR(math::ToFloat(result1.d), math::ToFloat(result2.d), tolerance);
    EXPECT_NEAR(math::ToFloat(result1.q), math::ToFloat(result2.q), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_small_values)
{
    float small_val;
    float rel_tolerance;

    if constexpr (std::is_same_v<TypeParam, math::Q15>)
    {
        small_val = 0.01f;
        rel_tolerance = 0.15f;
    }
    else if constexpr (std::is_same_v<TypeParam, math::Q31>)
    {
        small_val = 0.001f;
        rel_tolerance = 0.1f;
    }
    else
    {
        small_val = 0.001f;
        rel_tolerance = 0.05f;
    }

    auto input = CreateThreePhase<TypeParam>(small_val, -small_val / 2, -small_val / 2);
    auto angle = controllers::CreateNormalizedAngle<TypeParam>(M_PI / 4);

    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    auto IsErrorAcceptable = [rel_tolerance](float expected, float actual, const char* label) -> bool
    {
        float abs_diff = std::abs(actual - expected);
        float abs_expected = std::abs(expected);

        if constexpr (std::is_same_v<TypeParam, math::Q15>)
        {
            constexpr float q15_step = 1.0f / 32768.0f;

            if (abs_expected < 256 * q15_step)
                return abs_diff <= 2 * q15_step;
        }

        if (abs_expected < 1e-4f)
            return abs_diff < 1e-4f;

        float rel_error = abs_diff / abs_expected;
        return rel_error <= rel_tolerance;
    };

    EXPECT_TRUE(IsErrorAcceptable(math::ToFloat(input.a), math::ToFloat(result.a), "a value"));
    EXPECT_TRUE(IsErrorAcceptable(math::ToFloat(input.b), math::ToFloat(result.b), "b value"));
    EXPECT_TRUE(IsErrorAcceptable(math::ToFloat(input.c), math::ToFloat(result.c), "c value"));
}
