#include "math/Matrix.hpp"
#include <gtest/gtest.h>

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
