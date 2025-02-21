#include "numerical/math/QNumber.hpp"
#include <gtest/gtest.h>

template<typename QNumType>
class QNumberTest
    : public ::testing::Test
{
protected:
    using IntType = typename std::decay<decltype(std::declval<QNumType>().RawValue())>::type;
};

using QNumberTypes = ::testing::Types<math::Q31, math::Q15>;
TYPED_TEST_SUITE(QNumberTest, QNumberTypes);

TYPED_TEST(QNumberTest, DefaultConstructor)
{
    TypeParam num;
    EXPECT_EQ(num.RawValue(), 0);
    EXPECT_FLOAT_EQ(num.ToFloat(), 0.0f);
}

TYPED_TEST(QNumberTest, FloatConstructor)
{
    struct TestCase
    {
        float input;
        float expected;
    } testCases[] = {
        { -1.0f, -1.0f },
        { 0.5f, 0.5f },
        { -0.5f, -0.5f },
        { 0.25f, 0.25f }
    };

    for (const auto& testCase : testCases)
    {
        TypeParam num(testCase.input);
        EXPECT_NEAR(num.ToFloat(), testCase.expected, 1e-4f)
            << "Failed for input: " << testCase.input;
    }
}

TYPED_TEST(QNumberTest, RawValueConstructor)
{
    using IntType = typename std::decay<decltype(std::declval<TypeParam>().RawValue())>::type;
    IntType rawValue = 1024;
    TypeParam num(static_cast<IntType>(rawValue));
    EXPECT_EQ(num.RawValue(), rawValue);
}

TYPED_TEST(QNumberTest, Addition)
{
    TypeParam a(0.15f);
    TypeParam b(0.25f);
    TypeParam result = a + b;
    EXPECT_NEAR(result.ToFloat(), 0.40f, 1e-4f);
}

TYPED_TEST(QNumberTest, Subtraction)
{
    TypeParam a(0.50f);
    TypeParam b(0.20f);
    TypeParam result = a - b;
    EXPECT_NEAR(result.ToFloat(), 0.30f, 1e-4f);
}

TYPED_TEST(QNumberTest, Multiplication)
{
    TypeParam a(0.20f);
    TypeParam b(0.30f);
    TypeParam result = a * b;
    EXPECT_NEAR(result.ToFloat(), 0.06f, 1e-4f);
}

TYPED_TEST(QNumberTest, Division)
{
    TypeParam a(0.20f);
    TypeParam b(0.40f);
    TypeParam result = a / b;
    EXPECT_NEAR(result.ToFloat(), 0.50f, 1e-4f);
}

TYPED_TEST(QNumberTest, CompoundAddition)
{
    TypeParam a(0.15f);
    TypeParam b(0.25f);
    a += b;
    EXPECT_NEAR(a.ToFloat(), 0.40f, 1e-4f);
}

TYPED_TEST(QNumberTest, CompoundSubtraction)
{
    TypeParam a(0.50f);
    TypeParam b(0.20f);
    a -= b;
    EXPECT_NEAR(a.ToFloat(), 0.30f, 1e-4f);
}

TYPED_TEST(QNumberTest, CompoundMultiplication)
{
    TypeParam a(0.20f);
    TypeParam b(0.30f);
    a *= b;
    EXPECT_NEAR(a.ToFloat(), 0.06f, 1e-4f);
}

TYPED_TEST(QNumberTest, CompoundDivision)
{
    TypeParam a(0.20f);
    TypeParam b(0.40f);
    a /= b;
    EXPECT_NEAR(a.ToFloat(), 0.50f, 1e-4f);
}

TYPED_TEST(QNumberTest, UnaryPlus)
{
    TypeParam a(0.15f);
    TypeParam result = +a;
    EXPECT_NEAR(result.ToFloat(), 0.15f, 1e-4f);
}

TYPED_TEST(QNumberTest, UnaryNegation)
{
    TypeParam a(0.15f);
    TypeParam result = -a;
    EXPECT_NEAR(result.ToFloat(), -0.15f, 1e-4f);
}

TYPED_TEST(QNumberTest, EqualityComparison)
{
    TypeParam a(0.15f);
    TypeParam b(0.15f);
    TypeParam c(0.20f);

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TYPED_TEST(QNumberTest, LessThanComparison)
{
    TypeParam a(0.15f);
    TypeParam b(0.20f);

    EXPECT_TRUE(a < b);
    EXPECT_FALSE(b < a);
}

TYPED_TEST(QNumberTest, GreaterThanComparison)
{
    TypeParam a(0.20f);
    TypeParam b(0.15f);

    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}

TYPED_TEST(QNumberTest, FromDuration)
{
    auto duration = std::chrono::microseconds(1000);
    TypeParam num = TypeParam::FromDuration(duration);
    EXPECT_EQ(num.RawValue(), 1000);
}

TYPED_TEST(QNumberTest, FixedPointInteraction)
{
    TypeParam a(0.15f);

    auto rawVal = a.RawValue();
    TypeParam b(rawVal);

    EXPECT_EQ(a, b);
}

TYPED_TEST(QNumberTest, DivideByZeroHandling)
{
    TypeParam a(0.10f);
    TypeParam zero(0.0f);

    EXPECT_DEATH({
        TypeParam result = a / zero;
    },
        "");
}
