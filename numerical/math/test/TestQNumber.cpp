// Copyright 2024 Numerical Toolbox Contributors
// SPDX-License-Identifier: MIT

#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    template<typename QNumType>
    class QNumberTest
        : public ::testing::Test
    {};

    using QNumberTypes = ::testing::Types<math::Q31, math::Q15>;
    TYPED_TEST_SUITE(QNumberTest, QNumberTypes);
}

TYPED_TEST(QNumberTest, DefaultConstructorIsZero)
{
    TypeParam num;

    EXPECT_EQ(num.RawValue(), 0);
    EXPECT_FLOAT_EQ(num.ToFloat(), 0.0f);
}

TYPED_TEST(QNumberTest, FloatConstructorPositiveHalf)
{
    TypeParam num(0.5f);

    EXPECT_NEAR(num.ToFloat(), 0.5f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, FloatConstructorNegativeHalf)
{
    TypeParam num(-0.5f);

    EXPECT_NEAR(num.ToFloat(), -0.5f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, FloatConstructorQuarter)
{
    TypeParam num(0.25f);

    EXPECT_NEAR(num.ToFloat(), 0.25f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, FloatConstructorNegativeOne)
{
    TypeParam num(-1.0f);

    EXPECT_NEAR(num.ToFloat(), -1.0f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, RawValueConstructorPreservesRawValue)
{
    using IntType = typename std::decay<decltype(std::declval<TypeParam>().RawValue())>::type;
    IntType rawValue{ 1024 };
    TypeParam num(rawValue);

    EXPECT_EQ(num.RawValue(), rawValue);
}

TYPED_TEST(QNumberTest, QuantizationErrorBoundedByHalfUlp)
{
    using IntType = typename std::decay<decltype(std::declval<TypeParam>().RawValue())>::type;
    constexpr int fractionalBits = (sizeof(IntType) == 4) ? 31 : 15;
    const float halfUlp = 1.0f / static_cast<float>(1LL << fractionalBits);
    TypeParam num(0.3f);

    float quantizationError = std::abs(num.ToFloat() - 0.3f);

    EXPECT_LE(quantizationError, halfUlp);
}

TYPED_TEST(QNumberTest, Addition)
{
    TypeParam a(0.15f);
    TypeParam b(0.25f);

    TypeParam result = a + b;

    EXPECT_NEAR(result.ToFloat(), 0.40f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, Subtraction)
{
    TypeParam a(0.50f);
    TypeParam b(0.20f);

    TypeParam result = a - b;

    EXPECT_NEAR(result.ToFloat(), 0.30f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, Multiplication)
{
    TypeParam a(0.20f);
    TypeParam b(0.30f);

    TypeParam result = a * b;

    EXPECT_NEAR(result.ToFloat(), 0.06f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, Division)
{
    TypeParam a(0.20f);
    TypeParam b(0.40f);

    TypeParam result = a / b;

    EXPECT_NEAR(result.ToFloat(), 0.50f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, CompoundAddition)
{
    TypeParam a(0.15f);
    TypeParam b(0.25f);

    a += b;

    EXPECT_NEAR(a.ToFloat(), 0.40f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, CompoundSubtraction)
{
    TypeParam a(0.50f);
    TypeParam b(0.20f);

    a -= b;

    EXPECT_NEAR(a.ToFloat(), 0.30f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, CompoundMultiplication)
{
    TypeParam a(0.20f);
    TypeParam b(0.30f);

    a *= b;

    EXPECT_NEAR(a.ToFloat(), 0.06f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, CompoundDivision)
{
    TypeParam a(0.20f);
    TypeParam b(0.40f);

    a /= b;

    EXPECT_NEAR(a.ToFloat(), 0.50f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, UnaryPlus)
{
    TypeParam a(0.15f);

    TypeParam result = +a;

    EXPECT_NEAR(result.ToFloat(), 0.15f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, UnaryNegation)
{
    TypeParam a(0.15f);

    TypeParam result = -a;

    EXPECT_NEAR(result.ToFloat(), -0.15f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, AdditiveIdentity)
{
    TypeParam a(0.25f);
    TypeParam zero;

    TypeParam result = a + zero;

    EXPECT_NEAR(result.ToFloat(), 0.25f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, AdditiveInverse)
{
    TypeParam a(0.25f);
    TypeParam negA = -a;

    TypeParam result = a + negA;

    EXPECT_NEAR(result.ToFloat(), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, MultiplicationByZeroIsZero)
{
    TypeParam a(0.25f);
    TypeParam zero;

    TypeParam result = a * zero;

    EXPECT_NEAR(result.ToFloat(), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, ZeroDividedByNonzeroIsZero)
{
    TypeParam zero;
    TypeParam b(0.25f);

    TypeParam result = zero / b;

    EXPECT_NEAR(result.ToFloat(), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(QNumberTest, EqualityComparisonEqual)
{
    TypeParam a(0.15f);
    TypeParam b(0.15f);

    EXPECT_TRUE(a == b);
}

TYPED_TEST(QNumberTest, EqualityComparisonNotEqual)
{
    TypeParam a(0.15f);
    TypeParam c(0.20f);

    EXPECT_FALSE(a == c);
}

TYPED_TEST(QNumberTest, LessThanComparisonTrue)
{
    TypeParam a(0.15f);
    TypeParam b(0.20f);

    EXPECT_TRUE(a < b);
}

TYPED_TEST(QNumberTest, LessThanComparisonFalse)
{
    TypeParam a(0.20f);
    TypeParam b(0.15f);

    EXPECT_FALSE(a < b);
}

TYPED_TEST(QNumberTest, GreaterThanComparisonTrue)
{
    TypeParam a(0.20f);
    TypeParam b(0.15f);

    EXPECT_TRUE(a > b);
}

TYPED_TEST(QNumberTest, GreaterThanComparisonFalse)
{
    TypeParam a(0.15f);
    TypeParam b(0.20f);

    EXPECT_FALSE(a > b);
}

TYPED_TEST(QNumberTest, LessThanOrEqualComparisonLess)
{
    TypeParam a(0.15f);
    TypeParam b(0.20f);

    EXPECT_TRUE(a <= b);
}

TYPED_TEST(QNumberTest, LessThanOrEqualComparisonEqual)
{
    TypeParam a(0.15f);
    TypeParam b(0.15f);

    EXPECT_TRUE(a <= b);
}

TYPED_TEST(QNumberTest, GreaterThanOrEqualComparisonGreater)
{
    TypeParam a(0.20f);
    TypeParam b(0.15f);

    EXPECT_TRUE(a >= b);
}

TYPED_TEST(QNumberTest, GreaterThanOrEqualComparisonEqual)
{
    TypeParam a(0.20f);
    TypeParam b(0.20f);

    EXPECT_TRUE(a >= b);
}

TYPED_TEST(QNumberTest, FromDurationStoresRawCount)
{
    auto duration = std::chrono::microseconds(1000);

    TypeParam num = TypeParam::FromDuration(duration);

    EXPECT_EQ(num.RawValue(), 1000);
}

TYPED_TEST(QNumberTest, RawValueRoundtripPreservesValue)
{
    TypeParam a(0.15f);
    auto rawVal = a.RawValue();
    TypeParam b(rawVal);

    EXPECT_EQ(a, b);
}

TYPED_TEST(QNumberTest, DivideByZeroDies)
{
    TypeParam a(0.10f);
    TypeParam zero(0.0f);

    EXPECT_DEATH({ TypeParam result = a / zero; }, ""); // NOLINT
}
