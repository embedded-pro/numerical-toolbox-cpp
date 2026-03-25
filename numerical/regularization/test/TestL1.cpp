#include "numerical/regularization/L1.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>

namespace
{
    template<typename T>
    class TestL1
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Size = 4;

        regularization::L1<T, Size> regularization{ T(0.001f) };
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestL1, TestedTypes);
}

TYPED_TEST(TestL1, CalculateZeroVector)
{
    using Vector = typename regularization::L1<TypeParam, TestFixture::Size>::Vector;
    Vector parameters{};

    TypeParam result = TestFixture::regularization.Calculate(parameters);
    EXPECT_EQ(result, TypeParam(0.0f));
}

TYPED_TEST(TestL1, CalculateMixedValues)
{
    using Vector = typename regularization::L1<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;

    parameters[0] = TypeParam(0.025f);
    parameters[1] = TypeParam(-0.05f);
    parameters[2] = TypeParam(0.075f);
    parameters[3] = TypeParam(-0.1f);

    TypeParam expected = TypeParam(0.000244f);
    TypeParam result = TestFixture::regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), math::ToFloat(expected), 1e-3f);
}

TYPED_TEST(TestL1, DifferentLambdaValue)
{
    using Vector = typename regularization::L1<TypeParam, TestFixture::Size>::Vector;

    regularization::L1<TypeParam, TestFixture::Size> regularization(TypeParam(0.01f));

    Vector parameters;
    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.03f);
    parameters[3] = TypeParam(0.04f);

    TypeParam expected = TypeParam(0.001f);
    TypeParam result = regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), math::ToFloat(expected), 1e-3f);
}

TYPED_TEST(TestL1, GradientPositiveValues)
{
    using Vector = typename regularization::L1<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;
    parameters[0] = TypeParam(0.1f);
    parameters[1] = TypeParam(0.2f);
    parameters[2] = TypeParam(0.3f);
    parameters[3] = TypeParam(0.4f);

    Vector gradient = TestFixture::regularization.Gradient(parameters);

    for (std::size_t i = 0; i < TestFixture::Size; ++i)
        EXPECT_NEAR(math::ToFloat(gradient[i]), 0.001f, 1e-3f);
}

TYPED_TEST(TestL1, GradientMixedValues)
{
    using Vector = typename regularization::L1<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;
    parameters[0] = TypeParam(0.1f);
    parameters[1] = TypeParam(-0.2f);
    parameters[2] = TypeParam(0.0f);
    parameters[3] = TypeParam(-0.3f);

    Vector gradient = TestFixture::regularization.Gradient(parameters);

    EXPECT_NEAR(math::ToFloat(gradient[0]), 0.001f, 1e-3f);
    EXPECT_NEAR(math::ToFloat(gradient[1]), -0.001f, 1e-3f);
    EXPECT_NEAR(math::ToFloat(gradient[2]), 0.0f, 1e-3f);
    EXPECT_NEAR(math::ToFloat(gradient[3]), -0.001f, 1e-3f);
}
