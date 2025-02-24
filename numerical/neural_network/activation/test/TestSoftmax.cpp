#include "numerical/neural_network/activation/Softmax.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestSoftmax
        : public ::testing::Test
    {
    public:
        neural_network::Softmax<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSoftmax, TestedTypes);
}

TYPED_TEST(TestSoftmax, ForwardZeroInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.1f))), 0.905f, 0.001f);
}

TYPED_TEST(TestSoftmax, ForwardNegativeInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.5f))), 0.607f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.9f))), 0.407f, 0.001f);
}

TYPED_TEST(TestSoftmax, BackwardTinyValues)
{
    TypeParam x = TypeParam(-0.7f);
    TypeParam y = this->activation.Forward(x);

    EXPECT_NEAR(math::ToFloat(this->activation.Backward(x)),
        math::ToFloat(y * (TypeParam(0.9999f) - y)), 0.001f);
}

TYPED_TEST(TestSoftmax, BackwardNegativeInputs)
{
    for (float x = -0.9f; x <= -0.5f; x += 0.1f)
    {
        TypeParam input = TypeParam(x);
        TypeParam output = this->activation.Forward(input);
        TypeParam derivative = this->activation.Backward(input);
        TypeParam expected = output * (TypeParam(0.9999f) - output);

        EXPECT_NEAR(math::ToFloat(derivative), math::ToFloat(expected), 0.001f);
    }
}

TYPED_TEST(TestSoftmax, ComparativeTest)
{
    TypeParam x1 = TypeParam(-0.8f);
    TypeParam x2 = TypeParam(-0.7f);
    TypeParam y1 = this->activation.Forward(x1);
    TypeParam y2 = this->activation.Forward(x2);

    EXPECT_LT(math::ToFloat(y1), math::ToFloat(y2));
}
