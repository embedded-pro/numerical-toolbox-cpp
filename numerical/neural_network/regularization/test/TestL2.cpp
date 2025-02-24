#include "numerical/neural_network/regularization/L2.hpp"
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

        neural_network::L2<T, Size> regularization{ T(0.001f) };
    };

    using TestedTypes = ::testing::Types<float, math::Q31>;
    TYPED_TEST_SUITE(TestL2, TestedTypes);
}

TYPED_TEST(TestL2, CalculateZeroVector)
{
    using Vector = typename neural_network::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters{};

    TypeParam result = TestFixture::regularization.Calculate(parameters);
    EXPECT_EQ(result, TypeParam(0.0f));
}

TYPED_TEST(TestL2, CalculatePositiveValues)
{
    using Vector = typename neural_network::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;

    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.05f);
    parameters[3] = TypeParam(0.08f);

    float expectedFloat = 0.0000047f;
    TypeParam result = TestFixture::regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), expectedFloat, 1e-8f);
}

TYPED_TEST(TestL2, CalculateMixedValues)
{
    using Vector = typename neural_network::L2<TypeParam, TestFixture::Size>::Vector;
    Vector parameters;

    parameters[0] = TypeParam(0.025f);
    parameters[1] = TypeParam(-0.05f);
    parameters[2] = TypeParam(0.075f);
    parameters[3] = TypeParam(-0.09f);

    float expectedFloat = 0.000008425f;
    TypeParam result = TestFixture::regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), expectedFloat, 1e-7f);
}

TYPED_TEST(TestL2, DifferentLambdaValue)
{
    using Vector = typename neural_network::L2<TypeParam, TestFixture::Size>::Vector;

    neural_network::L2<TypeParam, TestFixture::Size> regularization(TypeParam(0.01f));

    Vector parameters;
    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.03f);
    parameters[3] = TypeParam(0.04f);

    float expectedFloat = 0.000015f;
    TypeParam result = regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), expectedFloat, 1e-7f);
}
