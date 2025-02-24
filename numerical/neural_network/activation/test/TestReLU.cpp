#include "numerical/neural_network/activation/ReLU.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestReLU
        : public ::testing::Test
    {
    public:
        neural_network::ReLU<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestReLU, TestedTypes);
}

TYPED_TEST(TestReLU, ForwardPositiveInput)
{
    EXPECT_EQ(this->activation.Forward(TypeParam(0.5f)), TypeParam(0.5f));
    EXPECT_EQ(this->activation.Forward(TypeParam(0.999f)), TypeParam(0.999f));
}

TYPED_TEST(TestReLU, ForwardNegativeInput)
{
    EXPECT_EQ(this->activation.Forward(TypeParam(-0.5f)), TypeParam(0.0f));
    EXPECT_EQ(this->activation.Forward(TypeParam(-0.999f)), TypeParam(0.0f));
}

TYPED_TEST(TestReLU, ForwardZeroInput)
{
    EXPECT_EQ(this->activation.Forward(TypeParam(0.0f)), TypeParam(0.0f));
}

TYPED_TEST(TestReLU, BackwardPositiveInput)
{
    EXPECT_EQ(this->activation.Backward(TypeParam(0.5f)), TypeParam(0.9999f));
    EXPECT_EQ(this->activation.Backward(TypeParam(0.999f)), TypeParam(0.9999f));
}

TYPED_TEST(TestReLU, BackwardNegativeInput)
{
    EXPECT_EQ(this->activation.Backward(TypeParam(-0.5f)), TypeParam(0.0f));
    EXPECT_EQ(this->activation.Backward(TypeParam(-0.999f)), TypeParam(0.0f));
}

TYPED_TEST(TestReLU, BackwardZeroInput)
{
    EXPECT_EQ(this->activation.Backward(TypeParam(0.0f)), TypeParam(0.0f));
}
