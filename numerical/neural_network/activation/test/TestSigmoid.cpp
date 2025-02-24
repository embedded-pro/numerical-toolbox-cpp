#include "numerical/neural_network/activation/Sigmoid.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestSigmoid
        : public ::testing::Test
    {
    public:
        neural_network::Sigmoid<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSigmoid, TestedTypes);
}

TYPED_TEST(TestSigmoid, ForwardZeroInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.0f))), 0.5f, 0.001f);
}

TYPED_TEST(TestSigmoid, ForwardPositiveInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.5f))), 0.622f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.999f))), 0.731f, 0.001f);
}

TYPED_TEST(TestSigmoid, ForwardNegativeInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.5f))), 0.378f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.999f))), 0.269f, 0.001f);
}

TYPED_TEST(TestSigmoid, BackwardZeroInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(0.0f))), 0.25f, 0.001f);
}

TYPED_TEST(TestSigmoid, BackwardPositiveInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(0.5f))), 0.235f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(0.999f))), 0.197f, 0.001f);
}

TYPED_TEST(TestSigmoid, BackwardNegativeInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(-0.5f))), 0.235f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(-0.999f))), 0.197f, 0.001f);
}
