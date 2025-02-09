#include "math/QNumber.hpp"
#include "math/Toeplitz.hpp"
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
                if (std::abs(math::ToFloat(a.at(i, j)) - math::ToFloat(b.at(i, j))) >= epsilon)
                    return false;
            }
        }
        return true;
    }

    template<typename T>
    class ToeplitzMatrixTest
        : public ::testing::Test
    {
    protected:
        static constexpr size_t N = 2;
        using ToeplitzType = math::ToeplitzMatrix<T, N>;
        using VectorType = math::Vector<T, N>;
        using MatrixType = math::Matrix<T, N, N>;

        static T MakeValue(float f)
        {
            return T(std::max(std::min(f, 0.1f), -0.1f));
        }

        VectorType MakeVector(float a, float b)
        {
            return VectorType{
                { MakeValue(a) },
                { MakeValue(b) }
            };
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
    TYPED_TEST_SUITE(ToeplitzMatrixTest, TestTypes);
}

TYPED_TEST(ToeplitzMatrixTest, DefaultConstructor)
{
    typename TestFixture::ToeplitzType t;
    auto full = t.ToFullMatrix();

    for (size_t i = 0; i < TestFixture::N; ++i)
        for (size_t j = 0; j < TestFixture::N; ++j)
            EXPECT_FLOAT_EQ(math::ToFloat(full.at(i, j)), 0.0f);
}

TYPED_TEST(ToeplitzMatrixTest, SymmetricConstructor)
{
    auto vec = this->MakeVector(0.02f, 0.01f);
    typename TestFixture::ToeplitzType t(vec);

    EXPECT_TRUE(t.IsSymmetric());

    auto expected = this->MakeMatrix(0.02f, 0.01f, 0.01f, 0.02f);
    EXPECT_TRUE(AreMatricesNear(t.ToFullMatrix(), expected));
}

TYPED_TEST(ToeplitzMatrixTest, GeneralConstructor)
{
    auto row = this->MakeVector(0.02f, 0.01f);
    auto col = this->MakeVector(0.02f, -0.01f);
    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_FALSE(t.IsSymmetric());

    auto expected = this->MakeMatrix(0.02f, 0.01f, -0.01f, 0.02f);
    EXPECT_TRUE(AreMatricesNear(t.ToFullMatrix(), expected));
}

TYPED_TEST(ToeplitzMatrixTest, VectorMultiplication)
{
    auto vec = this->MakeVector(0.02f, 0.01f);
    typename TestFixture::ToeplitzType t(vec);

    auto x = this->MakeVector(0.01f, 0.01f);
    auto result = t * x;

    auto expected = this->MakeVector(0.0003f, 0.0003f);

    for (size_t i = 0; i < TestFixture::N; ++i)
        EXPECT_NEAR(math::ToFloat(result.at(i, 0)), math::ToFloat(expected.at(i, 0)), 1e-4f);
}

TYPED_TEST(ToeplitzMatrixTest, Addition)
{
    auto vec1 = this->MakeVector(0.02f, 0.01f);
    auto vec2 = this->MakeVector(0.01f, 0.005f);

    typename TestFixture::ToeplitzType t1(vec1);
    typename TestFixture::ToeplitzType t2(vec2);

    auto result = t1 + t2;

    auto expected = this->MakeMatrix(0.03f, 0.015f, 0.015f, 0.03f);
    EXPECT_TRUE(AreMatricesNear(result.ToFullMatrix(), expected));
}

TYPED_TEST(ToeplitzMatrixTest, Subtraction)
{
    auto vec1 = this->MakeVector(0.02f, 0.01f);
    auto vec2 = this->MakeVector(0.01f, 0.005f);

    typename TestFixture::ToeplitzType t1(vec1);
    typename TestFixture::ToeplitzType t2(vec2);

    auto result = t1 - t2;

    auto expected = this->MakeMatrix(0.01f, 0.005f, 0.005f, 0.01f);
    EXPECT_TRUE(AreMatricesNear(result.ToFullMatrix(), expected));
}

TYPED_TEST(ToeplitzMatrixTest, ElementAccess)
{
    auto row = this->MakeVector(0.02f, 0.01f);
    auto col = this->MakeVector(0.02f, -0.01f);
    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_NEAR(math::ToFloat(t.at(0, 0)), 0.02f, 1e-4f);
    EXPECT_NEAR(math::ToFloat(t.at(0, 1)), 0.01f, 1e-4f);
    EXPECT_NEAR(math::ToFloat(t.at(1, 0)), -0.01f, 1e-4f);
    EXPECT_NEAR(math::ToFloat(t.at(1, 1)), 0.02f, 1e-4f);
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrix_ValidCase)
{
    auto matrix = this->MakeMatrix(
        0.02f, 0.01f, // First row:    [0.02, 0.01]
        0.03f, 0.02f  // Second row:   [0.03, 0.02]
    );

    EXPECT_TRUE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrix_InvalidCase)
{
    auto matrix = this->MakeMatrix(
        0.02f, 0.01f, // First row:    [0.02, 0.01]
        0.03f, 0.05f  // Second row:   [0.03, 0.05] - 0.05 breaks the pattern
    );

    EXPECT_FALSE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrix_ZeroMatrix)
{
    auto matrix = this->MakeMatrix(
        0.0f, 0.0f,
        0.0f, 0.0f);

    EXPECT_TRUE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectors_GeneralCase)
{
    auto matrix = this->MakeMatrix(
        0.02f, 0.01f, // First row:    [0.02, 0.01]
        0.03f, 0.02f  // Second row:   [0.03, 0.02]
    );

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    EXPECT_NEAR(math::ToFloat(row.at(0, 0)), 0.02f, 1e-4f);
    EXPECT_NEAR(math::ToFloat(row.at(1, 0)), 0.01f, 1e-4f);

    EXPECT_NEAR(math::ToFloat(col.at(0, 0)), 0.02f, 1e-4f);
    EXPECT_NEAR(math::ToFloat(col.at(1, 0)), 0.03f, 1e-4f);
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectors_SymmetricCase)
{
    auto matrix = this->MakeMatrix(
        0.02f, 0.01f, // First row:    [0.02, 0.01]
        0.01f, 0.02f  // Second row:   [0.01, 0.02]
    );

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    EXPECT_NEAR(math::ToFloat(row.at(0, 0)), math::ToFloat(col.at(0, 0)), 1e-4f);
    EXPECT_NEAR(math::ToFloat(row.at(1, 0)), math::ToFloat(col.at(1, 0)), 1e-4f);
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectors_ZeroMatrix)
{
    auto matrix = this->MakeMatrix(
        0.0f, 0.0f,
        0.0f, 0.0f);

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    for (size_t i = 0; i < TestFixture::N; ++i)
    {
        EXPECT_NEAR(math::ToFloat(row.at(i, 0)), 0.0f, 1e-4f);
        EXPECT_NEAR(math::ToFloat(col.at(i, 0)), 0.0f, 1e-4f);
    }
}
