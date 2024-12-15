#include <gtest/gtest.h>
#include "math/QNumber.hpp"
#include <limits>
#include <cmath>

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
    }
    testCases[] = 
    {
        {1.0f, 1.0f},
        {-1.0f, -1.0f},
        {0.5f, 0.5f},
        {-0.5f, -0.5f},
        {0.25f, 0.25f}
    };

    for (const auto& testCase : testCases) 
    {
        TypeParam num(testCase.input);
        EXPECT_NEAR(num.ToFloat(), testCase.expected, 1e-5f) 
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
    TypeParam a(1.5f);
    TypeParam b(2.5f);
    TypeParam result = a + b;
    EXPECT_NEAR(result.ToFloat(), 4.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, Subtraction)
{
    TypeParam a(5.0f);
    TypeParam b(2.0f);
    TypeParam result = a - b;
    EXPECT_NEAR(result.ToFloat(), 3.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, Multiplication) 
{
    TypeParam a(2.0f);
    TypeParam b(3.0f);
    TypeParam result = a * b;
    EXPECT_NEAR(result.ToFloat(), 6.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, Division)
{
    TypeParam a(6.0f);
    TypeParam b(2.0f);
    TypeParam result = a / b;
    EXPECT_NEAR(result.ToFloat(), 3.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, CompoundAddition) 
{
    TypeParam a(1.5f);
    TypeParam b(2.5f);
    a += b;
    EXPECT_NEAR(a.ToFloat(), 4.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, CompoundSubtraction) 
{
    TypeParam a(5.0f);
    TypeParam b(2.0f);
    a -= b;
    EXPECT_NEAR(a.ToFloat(), 3.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, CompoundMultiplication) 
{
    TypeParam a(2.0f);
    TypeParam b(3.0f);
    a *= b;
    EXPECT_NEAR(a.ToFloat(), 6.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, CompoundDivision) 
{
    TypeParam a(6.0f);
    TypeParam b(2.0f);
    a /= b;
    EXPECT_NEAR(a.ToFloat(), 3.0f, 1e-5f);
}

TYPED_TEST(QNumberTest, UnaryPlus) 
{
    TypeParam a(1.5f);
    TypeParam result = +a;
    EXPECT_NEAR(result.ToFloat(), 1.5f, 1e-5f);
}

TYPED_TEST(QNumberTest, UnaryNegation) 
{
    TypeParam a(1.5f);
    TypeParam result = -a;
    EXPECT_NEAR(result.ToFloat(), -1.5f, 1e-5f);
}

TYPED_TEST(QNumberTest, EqualityComparison) 
{
    TypeParam a(1.5f);
    TypeParam b(1.5f);
    TypeParam c(2.0f);
    
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TYPED_TEST(QNumberTest, LessThanComparison)
{
    TypeParam a(1.5f);
    TypeParam b(2.0f);
    
    EXPECT_TRUE(a < b);
    EXPECT_FALSE(b < a);
}

TYPED_TEST(QNumberTest, GreaterThanComparison) 
{
    TypeParam a(2.0f);
    TypeParam b(1.5f);
    
    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}

TYPED_TEST(QNumberTest, FromDuration)
{
    auto duration = std::chrono::microseconds(1000);
    TypeParam num = TypeParam::FromDuration(duration);
    EXPECT_EQ(num.RawValue(), 1000);
}

TYPED_TEST(QNumberTest, OverflowHandling) 
{
    using IntType = typename std::decay<decltype(std::declval<TypeParam>().RawValue())>::type;
    
    TypeParam maxVal(std::numeric_limits<float>::max() > 1.0f ? 1.0f : std::numeric_limits<float>::max());
    EXPECT_NO_THROW(maxVal.RawValue());

    TypeParam minVal(std::numeric_limits<float>::lowest() < -1.0f ? -1.0f : std::numeric_limits<float>::lowest());
    EXPECT_NO_THROW(minVal.RawValue());
}

TYPED_TEST(QNumberTest, FixedPointInteraction) 
{
    TypeParam a(1.5f);
    
    auto rawVal = a.RawValue();
    TypeParam b(rawVal);
    
    EXPECT_EQ(a, b);
}

TYPED_TEST(QNumberTest, DivideByZeroHandling)
{
    TypeParam a(1.0f);
    TypeParam zero(0.0f);
    
    EXPECT_DEATH({
        TypeParam result = a / zero;
    }, "");
}
