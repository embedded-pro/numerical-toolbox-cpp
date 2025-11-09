#include "numerical/neural_network/activation/LeakyReLU.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestLeakyReLU
        : public ::testing::Test
    {
    public:
        neural_network::LeakyReLU<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestLeakyReLU, TestedTypes);
}

TYPED_TEST(TestLeakyReLU, ForwardPositiveInput)
{
    EXPECT_EQ(this->activation.Forward(TypeParam(0.5f)), TypeParam(0.5f));
    EXPECT_EQ(this->activation.Forward(TypeParam(0.999f)), TypeParam(0.999f));
}

TYPED_TEST(TestLeakyReLU, ForwardNegativeInput)
{
    const auto alpha = TypeParam(0.01f);
    neural_network::LeakyReLU<TypeParam> activation(alpha);

    EXPECT_EQ(activation.Forward(TypeParam(-0.5f)), TypeParam(-0.005f));
}

TYPED_TEST(TestLeakyReLU, ForwardZeroInput)
{
    EXPECT_EQ(this->activation.Forward(TypeParam(0.0f)), TypeParam(0.0f));
}

TYPED_TEST(TestLeakyReLU, BackwardPositiveInput)
{
    EXPECT_EQ(this->activation.Backward(TypeParam(0.5f)), TypeParam(0.9999f));
    EXPECT_EQ(this->activation.Backward(TypeParam(0.999f)), TypeParam(0.9999f));
}

TYPED_TEST(TestLeakyReLU, BackwardNegativeInput)
{
    const auto alpha = TypeParam(0.01f);
    neural_network::LeakyReLU<TypeParam> activation(alpha);

    EXPECT_EQ(activation.Backward(TypeParam(-0.5f)), alpha);
    EXPECT_EQ(activation.Backward(TypeParam(-0.999f)), alpha);
}

TYPED_TEST(TestLeakyReLU, BackwardZeroInput)
{
    const auto alpha = TypeParam(0.01f);
    neural_network::LeakyReLU<TypeParam> activation(alpha);

    EXPECT_EQ(activation.Backward(TypeParam(0.0f)), alpha);
}
