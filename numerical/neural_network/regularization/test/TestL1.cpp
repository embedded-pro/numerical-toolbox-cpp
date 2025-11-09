#include "numerical/neural_network/regularization/L1.hpp"
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

        neural_network::L1<T, Size> regularization{ T(0.001f) };
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestL1, TestedTypes);
}

TYPED_TEST(TestL1, CalculateZeroVector)
{
    using Vector = typename neural_network::L1<TypeParam, TestFixture::Size>::Vector;
    Vector parameters{};

    TypeParam result = TestFixture::regularization.Calculate(parameters);
    EXPECT_EQ(result, TypeParam(0.0f));
}

TYPED_TEST(TestL1, CalculateMixedValues)
{
    using Vector = typename neural_network::L1<TypeParam, TestFixture::Size>::Vector;
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
    using Vector = typename neural_network::L1<TypeParam, TestFixture::Size>::Vector;

    neural_network::L1<TypeParam, TestFixture::Size> regularization(TypeParam(0.01f));

    Vector parameters;
    parameters[0] = TypeParam(0.01f);
    parameters[1] = TypeParam(0.02f);
    parameters[2] = TypeParam(0.03f);
    parameters[3] = TypeParam(0.04f);

    TypeParam expected = TypeParam(0.001f);
    TypeParam result = regularization.Calculate(parameters);

    EXPECT_NEAR(math::ToFloat(result), math::ToFloat(expected), 1e-3f);
}
