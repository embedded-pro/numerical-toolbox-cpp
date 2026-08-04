#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/ConditionNumber.hpp"
#include <gtest/gtest.h>

namespace
{
    class ConditionNumberTest : public ::testing::Test
    {
    protected:
        math::SquareMatrix<float, 2> a{
            { 3.0f, 1.0f },
            { 1.0f, 2.0f }
        };
    };
}

TEST_F(ConditionNumberTest, Identity)
{
    math::SquareMatrix<float, 2> identity{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    auto result = solvers::ConditionNumber(identity);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0f, math::Tolerance<float>());
}

TEST_F(ConditionNumberTest, WellConditioned2x2)
{
    auto result = solvers::ConditionNumber(a);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 3.2f, math::Tolerance<float>());
}

TEST_F(ConditionNumberTest, DiagonalScaling3x3)
{
    math::SquareMatrix<float, 3> diag{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 2.0f, 0.0f },
        { 0.0f, 0.0f, 4.0f }
    };
    auto result = solvers::ConditionNumber(diag);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 4.0f, math::Tolerance<float>());
}

TEST_F(ConditionNumberTest, IllConditionedHilbert3x3)
{
    math::SquareMatrix<float, 3> hilbert{
        { 1.0f,       0.5f,       1.0f / 3.0f },
        { 0.5f,       1.0f / 3.0f, 0.25f      },
        { 1.0f / 3.0f, 0.25f,     0.2f        }
    };
    auto result = solvers::ConditionNumber(hilbert);
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(*result, 100.0f);
}

TEST_F(ConditionNumberTest, ZeroRowReturnsNullopt)
{
    math::SquareMatrix<float, 2> singular{
        { 1.0f, 2.0f },
        { 0.0f, 0.0f }
    };
    auto result = solvers::ConditionNumber(singular);
    EXPECT_FALSE(result.has_value());
}

TEST_F(ConditionNumberTest, AllTinyRowReturnsNullopt)
{
    math::SquareMatrix<float, 2> nearZero{
        { 1.0f,  2.0f  },
        { 5e-13f, 5e-13f }
    };
    auto result = solvers::ConditionNumber(nearZero);
    EXPECT_FALSE(result.has_value());
}
