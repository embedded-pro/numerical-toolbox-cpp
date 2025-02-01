#include "math/ComplexNumber.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename QNumberType>
    class ComplexTest
        : public ::testing::Test
    {
    protected:
        using ComplexType = math::Complex<QNumberType>;
        static constexpr float kEpsilon = 1e-4f;

        static float ToFloat(const float& value)
        {
            return value;
        }

        static float ToFloat(const math::Q15& value)
        {
            return value.ToFloat();
        }

        static float ToFloat(const math::Q31& value)
        {
            return value.ToFloat();
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(ComplexTest, TestedTypes);
}

TYPED_TEST(ComplexTest, DefaultConstructor)
{
    typename TestFixture::ComplexType num;
    EXPECT_FLOAT_EQ(this->ToFloat(num.Real()), 0.0f);
    EXPECT_FLOAT_EQ(this->ToFloat(num.Imaginary()), 0.0f);
}

TYPED_TEST(ComplexTest, ComponentConstructor)
{
    TypeParam real(0.5f);
    TypeParam imag(0.3f);
    typename TestFixture::ComplexType num(real, imag);

    EXPECT_NEAR(this->ToFloat(num.Real()), 0.5f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(num.Imaginary()), 0.3f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, FloatConstructor)
{
    typename TestFixture::ComplexType num(0.5f, 0.3f);

    EXPECT_NEAR(this->ToFloat(num.Real()), 0.5f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(num.Imaginary()), 0.3f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, Addition)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    typename TestFixture::ComplexType result = a + b;

    EXPECT_NEAR(this->ToFloat(result.Real()), 0.4f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(result.Imaginary()), 0.6f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, Subtraction)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    typename TestFixture::ComplexType result = a - b;

    EXPECT_NEAR(this->ToFloat(result.Real()), 0.2f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(result.Imaginary()), 0.2f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, Multiplication)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    typename TestFixture::ComplexType result = a * b;

    // (0.3 + 0.4i)(0.1 + 0.2i) = (0.3*0.1 - 0.4*0.2) + (0.3*0.2 + 0.4*0.1)i
    EXPECT_NEAR(this->ToFloat(result.Real()), -0.05f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(result.Imaginary()), 0.10f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, CompoundAddition)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    a += b;

    EXPECT_NEAR(this->ToFloat(a.Real()), 0.4f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(a.Imaginary()), 0.6f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, CompoundSubtraction)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    a -= b;

    EXPECT_NEAR(this->ToFloat(a.Real()), 0.2f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(a.Imaginary()), 0.2f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, CompoundMultiplication)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.1f, 0.2f);
    a *= b;

    EXPECT_NEAR(this->ToFloat(a.Real()), -0.05f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(a.Imaginary()), 0.10f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, UnaryPlus)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType result = +a;

    EXPECT_NEAR(this->ToFloat(result.Real()), 0.3f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(result.Imaginary()), 0.4f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, UnaryNegation)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType result = -a;

    EXPECT_NEAR(this->ToFloat(result.Real()), -0.3f, TestFixture::kEpsilon);
    EXPECT_NEAR(this->ToFloat(result.Imaginary()), -0.4f, TestFixture::kEpsilon);
}

TYPED_TEST(ComplexTest, EqualityComparison)
{
    typename TestFixture::ComplexType a(0.3f, 0.4f);
    typename TestFixture::ComplexType b(0.3f, 0.4f);
    typename TestFixture::ComplexType c(0.3f, 0.5f);

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}
