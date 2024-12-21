#include "controllers/TransformsClarkePark.hpp"
#include <gmock/gmock.h>

namespace
{
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

    template<typename T>
    controllers::ThreePhase<T> CreateThreePhase(float a, float b, float c)
    {
        float scale = std::is_same_v<T, float> ? 1.0f : 0.2f;
        return {
            CreateValue<T>(scale * a),
            CreateValue<T>(scale * b),
            CreateValue<T>(scale * c)
        };
    }

    template<typename T>
    controllers::TwoPhase<T> CreateTwoPhase(float alpha, float beta)
    {
        float scale = std::is_same_v<T, float> ? 1.0f : 0.2f;
        return {
            CreateValue<T>(scale * alpha),
            CreateValue<T>(scale * beta)
        };
    }

    template<typename T>
    float GetTolerance()
    {
        if constexpr (std::is_same_v<T, float>)
            return 1e-3f;
        else if constexpr (std::is_same_v<T, math::Q31>)
            return 2e-2f;
        else
            return 3e-2f;
    }

    template<typename T>
    class TestTransforms
        : public ::testing::Test
    {
    public:
        std::optional<controllers::Clarke<T>> clarke;
        std::optional<controllers::Park<T>> park;
        std::optional<controllers::ClarkePark<T>> clarkePark;

        void SetUp() override
        {
            clarke.emplace(advancedFunctions);
            park.emplace(trigFunctions);
            clarkePark.emplace(trigFunctions, advancedFunctions);
        }

    private:
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
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestTransforms, TestedTypes);
}

TYPED_TEST(TestTransforms, clarke_balanced_system)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.25f, -0.25f);
    auto result = this->clarke->Forward(input);

    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.alpha), scale * 0.5f, tolerance);
    EXPECT_NEAR(ToFloat(result.beta), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, clarke_zero_input)
{
    auto input = CreateThreePhase<TypeParam>(0.0f, 0.0f, 0.0f);
    auto result = this->clarke->Forward(input);

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.alpha), 0.0f, tolerance);
    EXPECT_NEAR(ToFloat(result.beta), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, clarke_inverse_recovers_original)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.2f, -0.3f);
    auto alphabeta = this->clarke->Forward(input);
    auto result = this->clarke->Inverse(alphabeta);

    float tolerance = std::is_same_v<TypeParam, float> ? 0.03f : GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.a), ToFloat(input.a), tolerance);
    EXPECT_NEAR(ToFloat(result.b), ToFloat(input.b), tolerance);
    EXPECT_NEAR(ToFloat(result.c), ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, park_zero_angle)
{
    auto input = CreateTwoPhase<TypeParam>(0.3f, 0.0f);
    auto result = this->park->Forward(input, CreateValue<TypeParam>(0.0f));

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.d), ToFloat(input.alpha), tolerance);
    EXPECT_NEAR(ToFloat(result.q), 0.0f, tolerance);
}

TYPED_TEST(TestTransforms, park_ninety_degrees)
{
    auto input = CreateTwoPhase<TypeParam>(0.1f, 0.0f);
    auto result = this->park->Forward(input, CreateValue<TypeParam>(M_PI_2));

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.d), 0.0f, tolerance);
    EXPECT_NEAR(ToFloat(result.q), -ToFloat(input.alpha), tolerance);
}

TYPED_TEST(TestTransforms, park_inverse_recovers_original)
{
    auto input = CreateTwoPhase<TypeParam>(0.5f, 0.3f);
    auto angle = CreateValue<TypeParam>(M_PI / 4);
    auto dq = this->park->Forward(input, angle);
    auto result = this->park->Inverse(dq, angle);

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.alpha), ToFloat(input.alpha), tolerance);
    EXPECT_NEAR(ToFloat(result.beta), ToFloat(input.beta), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_full_transform)
{
    auto input = CreateThreePhase<TypeParam>(0.8f, -0.4f, -0.4f);
    auto angle = CreateValue<TypeParam>(M_PI / 6);
    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.a), ToFloat(input.a), tolerance);
    EXPECT_NEAR(ToFloat(result.b), ToFloat(input.b), tolerance);
    EXPECT_NEAR(ToFloat(result.c), ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_multiple_angles)
{
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.25f, -0.25f);
    std::vector<float> angles = { 0.0f, M_PI_4, M_PI_2, 3 * M_PI_4, M_PI };

    float tolerance = GetTolerance<TypeParam>();

    for (const auto& angle : angles)
    {
        auto dq = this->clarkePark->Forward(input, CreateValue<TypeParam>(angle));
        auto result = this->clarkePark->Inverse(dq, CreateValue<TypeParam>(angle));

        EXPECT_NEAR(ToFloat(result.a), ToFloat(input.a), tolerance);
        EXPECT_NEAR(ToFloat(result.b), ToFloat(input.b), tolerance);
        EXPECT_NEAR(ToFloat(result.c), ToFloat(input.c), tolerance);
    }
}

TYPED_TEST(TestTransforms, clarke_unbalanced_system)
{
    auto input = CreateThreePhase<TypeParam>(0.8f, -0.1f, -0.4f);
    auto result = this->clarke->Forward(input);

    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float tolerance = GetTolerance<TypeParam>();

    float expected_alpha = scale * 2.0f / 3.0f * (0.8f - 0.5f * (-0.1f - 0.4f));
    float expected_beta = scale * 0.866f * (-0.1f - (-0.4f));

    EXPECT_NEAR(ToFloat(result.alpha), expected_alpha, tolerance);
    EXPECT_NEAR(ToFloat(result.beta), expected_beta, tolerance);
}

TYPED_TEST(TestTransforms, park_negative_angles)
{
    auto input = CreateTwoPhase<TypeParam>(0.3f, 0.2f);
    auto result = this->park->Forward(input, CreateValue<TypeParam>(-M_PI_4));

    float tolerance = GetTolerance<TypeParam>();

    float cos45 = std::cos(-M_PI_4); // ≈ 0.707
    float sin45 = std::sin(-M_PI_4); // ≈ -0.707

    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float expected_d = scale * (0.3f * cos45 + 0.2f * sin45);
    float expected_q = scale * (-0.3f * sin45 + 0.2f * cos45);

    EXPECT_NEAR(ToFloat(result.d), expected_d, tolerance);
    EXPECT_NEAR(ToFloat(result.q), expected_q, tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_near_limits)
{
    float max_val = std::is_same_v<TypeParam, float> ? 0.9f : 0.18f;
    auto input = CreateThreePhase<TypeParam>(max_val, -max_val / 2, -max_val / 2);
    auto angle = CreateValue<TypeParam>(M_PI / 3); // 60 degrees

    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    float tolerance = GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.a), ToFloat(input.a), tolerance);
    EXPECT_NEAR(ToFloat(result.b), ToFloat(input.b), tolerance);
    EXPECT_NEAR(ToFloat(result.c), ToFloat(input.c), tolerance);
}

TYPED_TEST(TestTransforms, clarke_dc_offset)
{
    auto input = CreateThreePhase<TypeParam>(0.6f, 0.1f, 0.1f);
    auto result = this->clarke->Forward(input);

    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float tolerance = GetTolerance<TypeParam>();

    float expected_alpha = scale * 2.0f / 3.0f * (0.6f - 0.1f);
    float expected_beta = 0.0f;

    EXPECT_NEAR(ToFloat(result.alpha), expected_alpha, tolerance);
    EXPECT_NEAR(ToFloat(result.beta), expected_beta, tolerance);
}

TYPED_TEST(TestTransforms, park_harmonic_angle)
{
    auto input = CreateTwoPhase<TypeParam>(0.2f, 0.3f);
    float base_rads = M_PI / 6;
    float scale = 1.0f;
    float tolerance = GetTolerance<TypeParam>();

    if constexpr (std::is_same_v<TypeParam, math::Q31>)
    {
        scale = 0.2f;
        base_rads = M_PI / 12;
        tolerance = 0.03f;
    }

    input = CreateTwoPhase<TypeParam>(scale * 0.2f, scale * 0.3f);
    auto base_angle = CreateValue<TypeParam>(base_rads);

    float harmonic_rads = std::fmod(base_rads + 2 * M_PI, 2 * M_PI);
    auto harmonic_angle = CreateValue<TypeParam>(harmonic_rads);

    auto result1 = this->park->Forward(input, base_angle);
    auto result2 = this->park->Forward(input, harmonic_angle);

    EXPECT_NEAR(ToFloat(result1.d), ToFloat(result2.d), tolerance);
    EXPECT_NEAR(ToFloat(result1.q), ToFloat(result2.q), tolerance);
}

TYPED_TEST(TestTransforms, clarke_park_small_values)
{
    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float small_val;
    float rel_tolerance;

    if constexpr (std::is_same_v<TypeParam, math::Q15>)
    {
        small_val = scale * 0.01f;
        rel_tolerance = 0.15f; // 15% tolerance for Q15
    }
    else if constexpr (std::is_same_v<TypeParam, math::Q31>)
    {
        small_val = scale * 0.001f;
        rel_tolerance = 0.1f; // 10% tolerance for Q31
    }
    else
    {
        small_val = scale * 0.001f;
        rel_tolerance = 0.05f; // 5% tolerance for float
    }

    auto input = CreateThreePhase<TypeParam>(small_val, -small_val / 2, -small_val / 2);
    auto angle = CreateValue<TypeParam>(M_PI / 4);

    auto dq = this->clarkePark->Forward(input, angle);
    auto result = this->clarkePark->Inverse(dq, angle);

    auto check_error = [rel_tolerance](float expected, float actual, const char* label) -> bool
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

    EXPECT_TRUE(check_error(ToFloat(input.a), ToFloat(result.a), "a value"));
    EXPECT_TRUE(check_error(ToFloat(input.b), ToFloat(result.b), "b value"));
    EXPECT_TRUE(check_error(ToFloat(input.c), ToFloat(result.c), "c value"));
}
