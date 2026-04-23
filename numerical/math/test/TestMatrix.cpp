#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <type_traits>

namespace
{
    template<typename T>
    bool AreMatricesNear(const math::Matrix<T, 2, 2>& a,
        const math::Matrix<T, 2, 2>& b,
        float epsilon = 1e-4f)
    {
        for (size_t i = 0; i < 2; ++i)
        {
            for (size_t j = 0; j < 2; ++j)
            {
                if constexpr (std::is_same_v<T, float>)
                    return (std::abs(a.at(i, j) - b.at(i, j)) < epsilon);
                else
                    return (std::abs(a.at(i, j).ToFloat() - b.at(i, j).ToFloat()) < epsilon);
            }
        }
        return true;
    }

    template<typename T>
    class MatrixTest
        : public ::testing::Test
    {
    protected:
        using MatrixType = math::Matrix<T, 2, 2>;
        using VectorType = math::Vector<T, 2>;

        static T MakeValue(float f)
        {
            f = std::max(std::min(f, 0.9999f), -0.9999f);
            if constexpr (std::is_same_v<T, float>)
                return f;
            else
                return T(f);
        }

        MatrixType MakeMatrix(float a11, float a12, float a21, float a22)
        {
            return MatrixType{
                { MakeValue(a11), MakeValue(a12) },
                { MakeValue(a21), MakeValue(a22) }
            };
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(MatrixTest, TestTypes);
}

TYPED_TEST(MatrixTest, DefaultConstructor)
{
    typename TestFixture::MatrixType m;
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            if constexpr (std::is_same_v<TypeParam, float>)
                EXPECT_FLOAT_EQ(m.at(i, j), 0.0f);
            else
                EXPECT_FLOAT_EQ(m.at(i, j).ToFloat(), 0.0f);
        }
    }
}

TYPED_TEST(MatrixTest, InitializerListConstructor)
{
    auto m = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);

    float expected[2][2] = { { 0.1f, 0.2f }, { 0.3f, 0.4f } };
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            if constexpr (std::is_same_v<TypeParam, float>)
                EXPECT_NEAR(m.at(i, j), expected[i][j], 1e-4f);
            else
                EXPECT_NEAR(m.at(i, j).ToFloat(), expected[i][j], 1e-4f);
        }
    }
}

TYPED_TEST(MatrixTest, Addition)
{
    auto m1 = this->MakeMatrix(0.3f, 0.2f, 0.1f, 0.2f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.1f);
    auto result = m1 + m2;

    auto expected = this->MakeMatrix(0.4f, 0.4f, 0.4f, 0.3f);
    EXPECT_TRUE(AreMatricesNear(result, expected));
}

TYPED_TEST(MatrixTest, Subtraction)
{
    auto m1 = this->MakeMatrix(0.5f, 0.4f, 0.3f, 0.2f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.1f, 0.1f);
    auto result = m1 - m2;

    auto expected = this->MakeMatrix(0.4f, 0.2f, 0.2f, 0.1f);
    EXPECT_TRUE(AreMatricesNear(result, expected));
}

TYPED_TEST(MatrixTest, Multiplication)
{
    auto m1 = this->MakeMatrix(0.5f, 0.3f, 0.2f, 0.4f);
    auto m2 = this->MakeMatrix(0.2f, 0.3f, 0.4f, 0.2f);
    auto result = m1 * m2;

    auto expected = this->MakeMatrix(0.22f, 0.21f, 0.20f, 0.14f);
    EXPECT_TRUE(AreMatricesNear(result, expected));
}

TYPED_TEST(MatrixTest, ScalarMultiplication)
{
    auto m = this->MakeMatrix(0.5f, 0.4f, 0.3f, 0.2f);
    auto scalar = this->MakeValue(0.5f);
    auto result = m * scalar;

    auto expected = this->MakeMatrix(0.25f, 0.2f, 0.15f, 0.1f);
    EXPECT_TRUE(AreMatricesNear(result, expected));
}

TYPED_TEST(MatrixTest, Transpose)
{
    auto m = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);
    auto result = m.Transpose();

    auto expected = this->MakeMatrix(0.1f, 0.3f, 0.2f, 0.4f);
    EXPECT_TRUE(AreMatricesNear(result, expected));
}

TYPED_TEST(MatrixTest, Identity)
{
    auto identity = TestFixture::MatrixType::Identity();

    auto expected = this->MakeMatrix(0.9999f, 0.0f, 0.0f, 0.9999f);
    EXPECT_TRUE(AreMatricesNear(identity, expected));
}

TYPED_TEST(MatrixTest, RangeLimits)
{
    auto max_matrix = this->MakeMatrix(0.9999f, 0.9999f, 0.9999f, 0.9999f);
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            if constexpr (std::is_same_v<TypeParam, float>)
                EXPECT_NEAR(max_matrix.at(i, j), 0.9999f, 1e-4f);
            else
                EXPECT_NEAR(max_matrix.at(i, j).ToFloat(), 0.9999f, 1e-4f);
        }
    }

    auto min_matrix = this->MakeMatrix(-0.9999f, -0.9999f, -0.9999f, -0.9999f);
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            if constexpr (std::is_same_v<TypeParam, float>)
                EXPECT_NEAR(min_matrix.at(i, j), -0.9999f, 1e-4f);
            else
                EXPECT_NEAR(min_matrix.at(i, j).ToFloat(), -0.9999f, 1e-4f);
        }
    }
}

TYPED_TEST(MatrixTest, MultiplicationRangeCheck)
{
    auto m1 = this->MakeMatrix(0.5f, 0.5f, 0.5f, 0.5f);
    auto m2 = this->MakeMatrix(0.5f, 0.5f, 0.5f, 0.5f);
    auto result = m1 * m2;

    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            if constexpr (std::is_same_v<TypeParam, float>)
                EXPECT_LE(std::abs(result.at(i, j)), 0.9999f);
            else
                EXPECT_LE(std::abs(result.at(i, j).ToFloat()), 0.9999f);
        }
    }
}

namespace
{
    class MatrixBlockTest : public ::testing::Test
    {
    protected:
        math::Matrix<float, 4, 4> dest{};
        math::Matrix<float, 2, 2> src{
            { 0.1f, 0.2f },
            { 0.3f, 0.4f }
        };
    };
}

TEST_F(MatrixBlockTest, SetBlock_writes_correct_elements)
{
    dest.SetBlock(src, 1, 1);

    EXPECT_FLOAT_EQ(dest.at(1, 1), 0.1f);
    EXPECT_FLOAT_EQ(dest.at(1, 2), 0.2f);
    EXPECT_FLOAT_EQ(dest.at(2, 1), 0.3f);
    EXPECT_FLOAT_EQ(dest.at(2, 2), 0.4f);
    EXPECT_FLOAT_EQ(dest.at(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(dest.at(3, 3), 0.0f);
}

TEST_F(MatrixBlockTest, GetBlock_reads_correct_elements)
{
    math::Matrix<float, 4, 4> m{
        { 0.0f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.1f, 0.2f, 0.0f },
        { 0.0f, 0.3f, 0.4f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.0f }
    };

    auto block = m.GetBlock<2, 2>(1, 1);

    EXPECT_FLOAT_EQ(block.at(0, 0), 0.1f);
    EXPECT_FLOAT_EQ(block.at(0, 1), 0.2f);
    EXPECT_FLOAT_EQ(block.at(1, 0), 0.3f);
    EXPECT_FLOAT_EQ(block.at(1, 1), 0.4f);
}

TEST_F(MatrixBlockTest, GetColumn_reads_correct_column)
{
    math::Matrix<float, 3, 3> m{
        { 0.1f, 0.2f, 0.3f },
        { 0.4f, 0.5f, 0.6f },
        { 0.7f, 0.8f, 0.9f }
    };

    auto col = m.GetColumn(1);

    EXPECT_FLOAT_EQ(col.at(0, 0), 0.2f);
    EXPECT_FLOAT_EQ(col.at(1, 0), 0.5f);
    EXPECT_FLOAT_EQ(col.at(2, 0), 0.8f);
}

TEST_F(MatrixBlockTest, SetBlock_then_GetBlock_roundtrip)
{
    dest.SetBlock(src, 2, 2);
    auto result = dest.GetBlock<2, 2>(2, 2);

    EXPECT_FLOAT_EQ(result.at(0, 0), src.at(0, 0));
    EXPECT_FLOAT_EQ(result.at(0, 1), src.at(0, 1));
    EXPECT_FLOAT_EQ(result.at(1, 0), src.at(1, 0));
    EXPECT_FLOAT_EQ(result.at(1, 1), src.at(1, 1));
}
