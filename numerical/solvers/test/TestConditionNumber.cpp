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

TEST_F(ConditionNumberTest, WellConditioned)
{
    auto result = solvers::ConditionNumber(a);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 3.2f, math::Tolerance<float>());
}

TEST_F(ConditionNumberTest, SingularReturnsNullopt)
{
    math::SquareMatrix<float, 2> singular{
        { 1.0f, 2.0f },
        { 0.0f, 0.0f }
    };
    auto result = solvers::ConditionNumber(singular);
    EXPECT_FALSE(result.has_value());
}
