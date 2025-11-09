#include "numerical/neural_network/activation/Tanh.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestTanh
        : public ::testing::Test
    {
    public:
        neural_network::Tanh<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestTanh, TestedTypes);
}

TYPED_TEST(TestTanh, ForwardZeroInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.0f))), 0.0f, 0.001f);
}

TYPED_TEST(TestTanh, ForwardSmallPositiveInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.5f))), 0.462f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(0.9f))), 0.716f, 0.001f);
}

TYPED_TEST(TestTanh, ForwardSmallNegativeInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.5f))), -0.462f, 0.001f);
    EXPECT_NEAR(math::ToFloat(this->activation.Forward(TypeParam(-0.9f))), -0.716f, 0.001f);
}

TYPED_TEST(TestTanh, BackwardZeroInput)
{
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(TypeParam(0.0f))), 1.0f, 0.001f);
}

TYPED_TEST(TestTanh, BackwardSmallPositiveInput)
{
    TypeParam x1 = TypeParam(0.5f);
    TypeParam y1 = this->activation.Forward(x1);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(x1)),
        1.0f - math::ToFloat(y1) * math::ToFloat(y1), 0.001f);

    TypeParam x2 = TypeParam(0.8f);
    TypeParam y2 = this->activation.Forward(x2);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(x2)),
        1.0f - math::ToFloat(y2) * math::ToFloat(y2), 0.001f);
}

TYPED_TEST(TestTanh, BackwardSmallNegativeInput)
{
    TypeParam x1 = TypeParam(-0.5f);
    TypeParam y1 = this->activation.Forward(x1);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(x1)),
        1.0f - math::ToFloat(y1) * math::ToFloat(y1), 0.001f);

    TypeParam x2 = TypeParam(-0.8f);
    TypeParam y2 = this->activation.Forward(x2);
    EXPECT_NEAR(math::ToFloat(this->activation.Backward(x2)),
        1.0f - math::ToFloat(y2) * math::ToFloat(y2), 0.001f);
}

TYPED_TEST(TestTanh, SymmetryProperty)
{
    for (float x = 0.1f; x <= 0.9f; x += 0.2f)
    {
        TypeParam posInput = TypeParam(x);
        TypeParam negInput = TypeParam(-x);

        EXPECT_NEAR(math::ToFloat(this->activation.Forward(posInput)),
            -math::ToFloat(this->activation.Forward(negInput)), 0.001f);
    }
}
