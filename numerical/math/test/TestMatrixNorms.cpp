#include "numerical/math/MatrixNorms.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class MatrixNormsTest : public ::testing::Test
    {
    protected:
        math::Matrix<float, 2, 2> a{
            { 3.0f, 1.0f },
            { 1.0f, 2.0f }
        };
        math::Vector<float, 2> v{ { 3.0f }, { 4.0f } };
    };
}

TEST_F(MatrixNormsTest, FrobeniusNorm)
{
    float result = math::FrobeniusNorm(a);
    EXPECT_NEAR(result, 3.87298f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, OffDiagonalFrobeniusNorm)
{
    math::Matrix<float, 3, 3> m{
        { 5.0f, 3.0f, 0.0f },
        { -3.0f, 7.0f, 4.0f },
        { 0.0f, -4.0f, 9.0f }
    };

    float result = math::OffDiagonalFrobeniusNorm(m);
    EXPECT_NEAR(result, 7.0710678f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, OneNorm)
{
    float result = math::OneNorm(a);
    EXPECT_NEAR(result, 4.0f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, InfinityNorm)
{
    float result = math::InfinityNorm(a);
    EXPECT_NEAR(result, 4.0f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, VectorNorm)
{
    float result = math::VectorNorm(v);
    EXPECT_NEAR(result, 5.0f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, DotProduct)
{
    math::Vector<float, 2> w{ { 1.0f }, { 2.0f } };
    float result = math::DotProduct(v, w);
    EXPECT_NEAR(result, 11.0f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, DotProductWithSelfEqualsNormSquared)
{
    float dot = math::DotProduct(v, v);
    float norm = math::VectorNorm(v);
    EXPECT_NEAR(dot, norm * norm, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, NormalizeUnit)
{
    auto result = math::Normalize(v);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->at(0, 0), 0.6f, math::Tolerance<float>());
    EXPECT_NEAR(result->at(1, 0), 0.8f, math::Tolerance<float>());
}

TEST_F(MatrixNormsTest, NormalizeZeroVectorReturnsNullopt)
{
    math::Vector<float, 2> zero{ { 0.0f }, { 0.0f } };
    auto result = math::Normalize(zero);
    EXPECT_FALSE(result.has_value());
}
