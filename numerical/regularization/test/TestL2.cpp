#include "numerical/regularization/L2.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>

namespace
{
    template<typename T>
    class TestL2
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Size = 4;

        regularization::L2<T, Size> regularization{ T(0.001f) };
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestL2, TestedTypes);
}

TYPED_TEST(TestL2, CalculateZeroVector)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters{};

    TypeParam result = TestFixture::regularization.Calculate(parameters);
    EXPECT_EQ(result, TypeParam(0.0f));
}

TYPED_TEST(TestL2, CalculatePositiveValues)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;

    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.05f);
    parameters[3] = TypeParam(0.08f);

    float expectedFloat = 0.0000047f;
    TypeParam result = TestFixture::regularization.Calculate(parameters);

    float tolerance = std::is_same_v<TypeParam, math::Q15> ? 1e-4f : 1e-8f;
    EXPECT_NEAR(math::ToFloat(result), expectedFloat, tolerance);
}

TYPED_TEST(TestL2, CalculateMixedValues)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;

    parameters[0] = TypeParam(0.025f);
    parameters[1] = TypeParam(-0.05f);
    parameters[2] = TypeParam(0.075f);
    parameters[3] = TypeParam(-0.09f);

    float expectedFloat = 0.000008425f;
    TypeParam result = TestFixture::regularization.Calculate(parameters);

    float tolerance = std::is_same_v<TypeParam, math::Q15> ? 1e-4f : 1e-7f;
    EXPECT_NEAR(math::ToFloat(result), expectedFloat, tolerance);
}

TYPED_TEST(TestL2, DifferentLambdaValue)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;

    regularization::L2<TypeParam, TestFixture::Size> regularization(TypeParam(0.01f));

    Vector parameters;
    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.03f);
    parameters[3] = TypeParam(0.04f);

    float expectedFloat = 0.000015f;
    TypeParam result = regularization.Calculate(parameters);

    float tolerance = std::is_same_v<TypeParam, math::Q15> ? 1e-4f : 1e-7f;
    EXPECT_NEAR(math::ToFloat(result), expectedFloat, tolerance);
}

TYPED_TEST(TestL2, GradientPositiveValues)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;
    parameters[0] = TypeParam(0.1f);
    parameters[1] = TypeParam(0.2f);
    parameters[2] = TypeParam(0.3f);
    parameters[3] = TypeParam(0.4f);

    Vector gradient = TestFixture::regularization.Gradient(parameters);

    for (std::size_t i = 0; i < TestFixture::Size; ++i)
        EXPECT_NEAR(math::ToFloat(gradient[i]), math::ToFloat(TypeParam(0.001f) * parameters[i]), 1e-3f);
}

TYPED_TEST(TestL2, GradientMixedValues)
{
    using Vector = typename regularization::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;
    parameters[0] = TypeParam(0.1f);
    parameters[1] = TypeParam(-0.2f);
    parameters[2] = TypeParam(0.0f);
    parameters[3] = TypeParam(-0.3f);

    Vector gradient = TestFixture::regularization.Gradient(parameters);

    for (std::size_t i = 0; i < TestFixture::Size; ++i)
        EXPECT_NEAR(math::ToFloat(gradient[i]), math::ToFloat(TypeParam(0.001f) * parameters[i]), 1e-3f);
}
