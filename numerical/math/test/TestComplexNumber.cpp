#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestComplexNumber
        : public ::testing::Test
    {
    protected:
        using ComplexType = math::Complex<float>;
    };
}

TEST_F(TestComplexNumber, DefaultConstructorIsZero)
{
    ComplexType num;

    EXPECT_NEAR(num.Real(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(num.Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, ComponentConstructorStoresValues)
{
    ComplexType num(0.5f, 0.3f);

    EXPECT_NEAR(num.Real(), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(num.Imaginary(), 0.3f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, Addition)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    ComplexType result = a + b;

    EXPECT_NEAR(result.Real(), 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.6f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, Subtraction)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    ComplexType result = a - b;

    EXPECT_NEAR(result.Real(), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.2f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, Multiplication)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    ComplexType result = a * b;

    EXPECT_NEAR(result.Real(), -0.05f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.10f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, Division)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    ComplexType result = a / b;

    EXPECT_NEAR(result.Real(), 2.2f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, DivisionByPureRealDivisor)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(2.0f, 0.0f);

    ComplexType result = a / b;

    EXPECT_NEAR(result.Real(), 0.15f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.2f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, DivisionByPureImaginaryDivisor)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.0f, 1.0f);

    ComplexType result = a / b;

    EXPECT_NEAR(result.Real(), 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.3f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, MultiplyThenDivideRoundTripRecoversOperand)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    ComplexType result = (a * b) / b;

    EXPECT_NEAR(result.Real(), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, AdditiveIdentity)
{
    ComplexType a(0.3f, -0.4f);
    ComplexType zero;

    ComplexType result = a + zero;

    EXPECT_NEAR(result.Real(), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, MultiplicativeIdentity)
{
    ComplexType a(0.3f, -0.4f);
    ComplexType one(1.0f, 0.0f);

    ComplexType result = a * one;

    EXPECT_NEAR(result.Real(), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, ImaginaryUnitSquaredIsNegativeOne)
{
    ComplexType i(0.0f, 1.0f);

    ComplexType result = i * i;

    EXPECT_NEAR(result.Real(), -1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, AdditiveInverseYieldsZero)
{
    ComplexType a(0.7f, -0.5f);

    ComplexType result = a + (-a);

    EXPECT_NEAR(result.Real(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, MultiplicationIsCommutative)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.5f, -0.2f);

    ComplexType ab = a * b;
    ComplexType ba = b * a;

    EXPECT_NEAR(ab.Real(), ba.Real(), math::Tolerance<float>());
    EXPECT_NEAR(ab.Imaginary(), ba.Imaginary(), math::Tolerance<float>());
}

TEST_F(TestComplexNumber, MultiplicationDistributesOverAddition)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);
    ComplexType c(0.5f, -0.1f);

    ComplexType lhs = a * (b + c);
    ComplexType rhs = a * b + a * c;

    EXPECT_NEAR(lhs.Real(), rhs.Real(), math::Tolerance<float>());
    EXPECT_NEAR(lhs.Imaginary(), rhs.Imaginary(), math::Tolerance<float>());
}

TEST_F(TestComplexNumber, AbsMatchesHypot)
{
    ComplexType a(3.0f, 4.0f);

    float result = math::Abs(a);

    EXPECT_NEAR(result, 5.0f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, AbsOfPureReal)
{
    ComplexType a(-2.5f, 0.0f);

    float result = math::Abs(a);

    EXPECT_NEAR(result, 2.5f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, AbsOfZeroIsZero)
{
    ComplexType zero;

    float result = math::Abs(zero);

    EXPECT_NEAR(result, 0.0f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, ModulusSquaredEqualsSumOfSquares)
{
    ComplexType a(3.0f, 4.0f);

    float modulus = math::Abs(a);
    float expected = std::sqrt(a.Real() * a.Real() + a.Imaginary() * a.Imaginary());

    EXPECT_NEAR(modulus, expected, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, CompoundAddition)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    a += b;

    EXPECT_NEAR(a.Real(), 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(a.Imaginary(), 0.6f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, CompoundSubtraction)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    a -= b;

    EXPECT_NEAR(a.Real(), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(a.Imaginary(), 0.2f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, CompoundMultiplication)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.1f, 0.2f);

    a *= b;

    EXPECT_NEAR(a.Real(), -0.05f, math::Tolerance<float>());
    EXPECT_NEAR(a.Imaginary(), 0.10f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, UnaryPlus)
{
    ComplexType a(0.3f, 0.4f);

    ComplexType result = +a;

    EXPECT_NEAR(result.Real(), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, UnaryNegation)
{
    ComplexType a(0.3f, 0.4f);

    ComplexType result = -a;

    EXPECT_NEAR(result.Real(), -0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.4f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, DoubleNegationIsIdentity)
{
    ComplexType a(0.3f, -0.7f);

    ComplexType result = -(-a);

    EXPECT_NEAR(result.Real(), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), -0.7f, math::Tolerance<float>());
}

TEST_F(TestComplexNumber, EqualityHoldsForIdenticalComponents)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType b(0.3f, 0.4f);

    EXPECT_TRUE(a == b);
}

TEST_F(TestComplexNumber, EqualityFailsForDifferentImaginaryComponent)
{
    ComplexType a(0.3f, 0.4f);
    ComplexType c(0.3f, 0.5f);

    EXPECT_FALSE(a == c);
}
