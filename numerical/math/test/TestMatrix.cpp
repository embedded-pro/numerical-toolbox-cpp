#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>

namespace
{
    using math::test::AreMatricesNear;
    using math::test::AreVectorsNear;

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

TYPED_TEST(MatrixTest, DefaultConstructorZeroInitializes)
{
    typename TestFixture::MatrixType m;

    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(math::ToFloat(m.at(i, j)), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, InitializerListConstructorStoresValues)
{
    auto m = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);

    EXPECT_NEAR(math::ToFloat(m.at(0, 0)), 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(0, 1)), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(1, 0)), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(1, 1)), 0.4f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, Addition)
{
    auto m1 = this->MakeMatrix(0.3f, 0.2f, 0.1f, 0.2f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.1f);

    auto result = m1 + m2;

    EXPECT_TRUE(AreMatricesNear(result, this->MakeMatrix(0.4f, 0.4f, 0.4f, 0.3f)));
}

TYPED_TEST(MatrixTest, Subtraction)
{
    auto m1 = this->MakeMatrix(0.5f, 0.4f, 0.3f, 0.2f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.1f, 0.1f);

    auto result = m1 - m2;

    EXPECT_TRUE(AreMatricesNear(result, this->MakeMatrix(0.4f, 0.2f, 0.2f, 0.1f)));
}

TYPED_TEST(MatrixTest, Multiplication)
{
    auto m1 = this->MakeMatrix(0.5f, 0.3f, 0.2f, 0.4f);
    auto m2 = this->MakeMatrix(0.2f, 0.3f, 0.4f, 0.2f);

    auto result = m1 * m2;

    EXPECT_TRUE(AreMatricesNear(result, this->MakeMatrix(0.22f, 0.21f, 0.20f, 0.14f)));
}

TYPED_TEST(MatrixTest, ScalarMultiplication)
{
    auto m = this->MakeMatrix(0.5f, 0.4f, 0.3f, 0.2f);
    auto scalar = this->MakeValue(0.5f);

    auto result = m * scalar;

    EXPECT_TRUE(AreMatricesNear(result, this->MakeMatrix(0.25f, 0.2f, 0.15f, 0.1f)));
}

TYPED_TEST(MatrixTest, Transpose)
{
    auto m = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);

    auto result = m.Transpose();

    EXPECT_TRUE(AreMatricesNear(result, this->MakeMatrix(0.1f, 0.3f, 0.2f, 0.4f)));
}

TYPED_TEST(MatrixTest, TransposeOfTransposeIsIdentity)
{
    auto m = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);

    auto result = m.Transpose().Transpose();

    EXPECT_TRUE(AreMatricesNear(result, m));
}

TYPED_TEST(MatrixTest, IdentityDiagonalIsOneOffDiagonalIsZero)
{
    auto identity = TestFixture::MatrixType::Identity();

    constexpr float expectedDiag = std::is_floating_point_v<TypeParam> ? 1.0f : 0.9999f;
    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
        {
            float expected = (i == j) ? expectedDiag : 0.0f;
            EXPECT_NEAR(math::ToFloat(identity.at(i, j)), expected, math::Tolerance<float>());
        }
}

TYPED_TEST(MatrixTest, AdditionAssignAccumulatesInPlace)
{
    auto m1 = this->MakeMatrix(0.2f, 0.1f, 0.3f, 0.1f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.1f, 0.2f);

    m1 += m2;

    EXPECT_TRUE(AreMatricesNear(m1, this->MakeMatrix(0.3f, 0.3f, 0.4f, 0.3f)));
}

TYPED_TEST(MatrixTest, SubtractionAssignDecreasesInPlace)
{
    auto m1 = this->MakeMatrix(0.4f, 0.3f, 0.5f, 0.3f);
    auto m2 = this->MakeMatrix(0.1f, 0.2f, 0.1f, 0.2f);

    m1 -= m2;

    EXPECT_TRUE(AreMatricesNear(m1, this->MakeMatrix(0.3f, 0.1f, 0.4f, 0.1f)));
}

TYPED_TEST(MatrixTest, ScalarMultiplyAssignScalesInPlace)
{
    auto m = this->MakeMatrix(0.4f, 0.2f, 0.6f, 0.2f);
    auto scalar = this->MakeValue(0.5f);

    m *= scalar;

    EXPECT_TRUE(AreMatricesNear(m, this->MakeMatrix(0.2f, 0.1f, 0.3f, 0.1f)));
}

TYPED_TEST(MatrixTest, TraceEqualsSumOfDiagonal)
{
    auto m = this->MakeMatrix(0.3f, 0.1f, 0.2f, 0.5f);

    auto trace = m.Trace();

    EXPECT_NEAR(math::ToFloat(trace), 0.8f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, MaxRangeValuesStoredCorrectly)
{
    auto max_matrix = this->MakeMatrix(0.9999f, 0.9999f, 0.9999f, 0.9999f);

    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(math::ToFloat(max_matrix.at(i, j)), 0.9999f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, MinRangeValuesStoredCorrectly)
{
    auto min_matrix = this->MakeMatrix(-0.9999f, -0.9999f, -0.9999f, -0.9999f);

    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(math::ToFloat(min_matrix.at(i, j)), -0.9999f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, AdditionIsCommutative)
{
    auto m1 = this->MakeMatrix(0.1f, 0.3f, 0.2f, 0.4f);
    auto m2 = this->MakeMatrix(0.3f, 0.1f, 0.4f, 0.2f);

    EXPECT_TRUE(AreMatricesNear(m1 + m2, m2 + m1));
}

TYPED_TEST(MatrixTest, PartialInitializerListZeroFillsRemainder)
{
    typename TestFixture::MatrixType m{
        { this->MakeValue(0.5f) }
    };

    EXPECT_NEAR(math::ToFloat(m.at(0, 0)), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(0, 1)), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(1, 0)), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(m.at(1, 1)), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(MatrixTest, MultiplicationByZeroMatrixYieldsZero)
{
    auto m = this->MakeMatrix(0.5f, 0.3f, 0.2f, 0.4f);
    typename TestFixture::MatrixType zero{};

    auto result = m * zero;

    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(math::ToFloat(result.at(i, j)), 0.0f, math::Tolerance<float>());
}

TEST_F(MatrixBlockTest, SetBlockWritesCorrectElements)
{
    dest.SetBlock(src, 1, 1);

    EXPECT_FLOAT_EQ(dest.at(1, 1), 0.1f);
    EXPECT_FLOAT_EQ(dest.at(1, 2), 0.2f);
    EXPECT_FLOAT_EQ(dest.at(2, 1), 0.3f);
    EXPECT_FLOAT_EQ(dest.at(2, 2), 0.4f);
    EXPECT_FLOAT_EQ(dest.at(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(dest.at(3, 3), 0.0f);
}

TEST_F(MatrixBlockTest, GetBlockReadsCorrectElements)
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

TEST_F(MatrixBlockTest, GetColumnReadsCorrectColumn)
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

TEST_F(MatrixBlockTest, SetBlockThenGetBlockRoundtrip)
{
    dest.SetBlock(src, 2, 2);
    auto result = dest.GetBlock<2, 2>(2, 2);

    EXPECT_FLOAT_EQ(result.at(0, 0), src.at(0, 0));
    EXPECT_FLOAT_EQ(result.at(0, 1), src.at(0, 1));
    EXPECT_FLOAT_EQ(result.at(1, 0), src.at(1, 0));
    EXPECT_FLOAT_EQ(result.at(1, 1), src.at(1, 1));
}

TEST_F(MatrixBlockTest, SetBlockDoesNotModifyUnaffectedElements)
{
    dest.SetBlock(src, 0, 0);

    EXPECT_FLOAT_EQ(dest.at(2, 2), 0.0f);
    EXPECT_FLOAT_EQ(dest.at(3, 3), 0.0f);
    EXPECT_FLOAT_EQ(dest.at(0, 2), 0.0f);
    EXPECT_FLOAT_EQ(dest.at(2, 0), 0.0f);
}

TEST_F(MatrixBlockTest, GetColumnFirstColumnMatchesFirstColumn)
{
    math::Matrix<float, 3, 3> m{
        { 0.1f, 0.4f, 0.7f },
        { 0.2f, 0.5f, 0.8f },
        { 0.3f, 0.6f, 0.9f }
    };

    auto col = m.GetColumn(0);

    EXPECT_FLOAT_EQ(col.at(0, 0), 0.1f);
    EXPECT_FLOAT_EQ(col.at(1, 0), 0.2f);
    EXPECT_FLOAT_EQ(col.at(2, 0), 0.3f);
}
