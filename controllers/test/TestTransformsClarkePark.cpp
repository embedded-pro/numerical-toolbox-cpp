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
        // Scale values for fixed-point types - using 0.2 to avoid overflow
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
    // Using normalized values that maintain the 120° relationship
    // For balanced system: a = 0.5, b = c = -0.25
    auto input = CreateThreePhase<TypeParam>(0.5f, -0.25f, -0.25f);
    auto result = this->clarke->Forward(input);

    float scale = std::is_same_v<TypeParam, float> ? 1.0f : 0.2f;
    float tolerance = GetTolerance<TypeParam>();

    // For balanced system, with a = 0.5*scale, b = c = -0.25*scale:
    // alpha = 2/3 * (a - 0.5*(b + c))
    //       = 2/3 * 0.5*scale
    //       = 0.5*scale
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

    // Use appropriate tolerance for each type:
    // - Float needs larger tolerance due to accumulated floating-point errors
    // - Fixed point types can use stricter tolerance
    float tolerance = std::is_same_v<TypeParam, float> ? 0.03f : GetTolerance<TypeParam>();

    EXPECT_NEAR(ToFloat(result.a), ToFloat(input.a), tolerance)
        << "a: expected " << ToFloat(input.a) << " got " << ToFloat(result.a)
        << " (type=" << typeid(TypeParam).name() << ")";
    EXPECT_NEAR(ToFloat(result.b), ToFloat(input.b), tolerance)
        << "b: expected " << ToFloat(input.b) << " got " << ToFloat(result.b)
        << " (type=" << typeid(TypeParam).name() << ")";
    EXPECT_NEAR(ToFloat(result.c), ToFloat(input.c), tolerance)
        << "c: expected " << ToFloat(input.c) << " got " << ToFloat(result.c)
        << " (type=" << typeid(TypeParam).name() << ")";
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
    // For 90 degrees, the transform acts as a rotation matrix:
    // [ cos(90°)  sin(90°)] = [ 0  1]
    // [-sin(90°)  cos(90°)]   [-1  0]

    // Using small values to prevent precision issues
    auto input = CreateTwoPhase<TypeParam>(0.1f, 0.0f);
    auto result = this->park->Forward(input, CreateValue<TypeParam>(M_PI_2));

    float tolerance = GetTolerance<TypeParam>();

    // At 90 degrees:
    // d = α*cos(90°) + β*sin(90°) = 0
    // q = -α*sin(90°) + β*cos(90°) = -α
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
