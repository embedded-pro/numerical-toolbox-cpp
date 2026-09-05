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

TYPED_TEST(MatrixTest, iterators_traverse_all_elements_in_row_major_order)
{
    auto matrix = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);
    const auto& constMatrix = matrix;

    std::size_t count = 0;
    for (auto it = matrix.begin(); it != matrix.end(); ++it)
        ++count;
    EXPECT_EQ(count, 4u);

    count = 0;
    for (auto it = constMatrix.begin(); it != constMatrix.end(); ++it)
        ++count;
    EXPECT_EQ(count, 4u);

    EXPECT_NEAR(math::ToFloat(*matrix.begin()), math::ToFloat(matrix.at(0, 0)), 1e-3f);
}

TYPED_TEST(MatrixTest, row_indexing_reaches_first_element_of_each_row)
{
    auto matrix = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);
    const auto& constMatrix = matrix;

    EXPECT_NEAR(math::ToFloat(matrix[0]), math::ToFloat(matrix.at(0, 0)), 1e-3f);
    EXPECT_NEAR(math::ToFloat(matrix[1]), math::ToFloat(matrix.at(1, 0)), 1e-3f);
    EXPECT_NEAR(math::ToFloat(constMatrix[1]), math::ToFloat(constMatrix.at(1, 0)), 1e-3f);
}

TYPED_TEST(MatrixTest, compound_assignment_matches_binary_operators)
{
    auto lhs = this->MakeMatrix(0.4f, 0.1f, 0.2f, 0.3f);
    auto rhs = this->MakeMatrix(0.1f, 0.1f, 0.1f, 0.1f);

    auto sum = lhs;
    sum += rhs;
    EXPECT_TRUE(AreMatricesNear(sum, lhs + rhs, math::Tolerance<TypeParam>()));

    auto difference = lhs;
    difference -= rhs;
    EXPECT_TRUE(AreMatricesNear(difference, lhs - rhs, math::Tolerance<TypeParam>()));

    auto scaled = lhs;
    scaled *= this->MakeValue(0.5f);
    EXPECT_TRUE(AreMatricesNear(scaled, lhs * this->MakeValue(0.5f), math::Tolerance<TypeParam>()));
}

TYPED_TEST(MatrixTest, transpose_and_trace_agree_with_definition)
{
    auto matrix = this->MakeMatrix(0.1f, 0.2f, 0.3f, 0.4f);
    auto transposed = matrix.Transpose();

    EXPECT_NEAR(math::ToFloat(transposed.at(0, 1)), math::ToFloat(matrix.at(1, 0)), 1e-3f);
    EXPECT_NEAR(math::ToFloat(transposed.at(1, 0)), math::ToFloat(matrix.at(0, 1)), 1e-3f);
    EXPECT_NEAR(math::ToFloat(matrix.Trace()),
        math::ToFloat(matrix.at(0, 0)) + math::ToFloat(matrix.at(1, 1)), 1e-3f);
}

namespace
{
    class MatrixFloatShapeTest : public ::testing::Test
    {
    protected:
        math::Matrix<float, 3, 3> square3{
            { 1.0f, 2.0f, 3.0f },
            { 4.0f, 5.0f, 6.0f },
            { 7.0f, 8.0f, 10.0f }
        };
    };
}

TEST_F(MatrixFloatShapeTest, identity_is_diagonal_with_unit_trace_per_dimension)
{
    auto identity2 = math::SquareMatrix<float, 2>::Identity();
    EXPECT_NEAR(identity2.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(identity2.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(identity2.Trace(), 2.0f, math::Tolerance<float>());

    auto identity3 = math::SquareMatrix<float, 3>::Identity();
    EXPECT_NEAR(identity3.Trace(), 3.0f, math::Tolerance<float>());
}

TEST_F(MatrixFloatShapeTest, three_by_three_iterators_and_row_access)
{
    const auto& constSquare = square3;

    std::size_t count = 0;
    for (auto it = square3.begin(); it != square3.end(); ++it)
        ++count;
    EXPECT_EQ(count, 9u);

    count = 0;
    for (auto it = constSquare.begin(); it != constSquare.end(); ++it)
        ++count;
    EXPECT_EQ(count, 9u);

    EXPECT_NEAR(square3[2], 7.0f, math::Tolerance<float>());
    EXPECT_NEAR(constSquare[1], 4.0f, math::Tolerance<float>());
}

TEST_F(MatrixFloatShapeTest, three_by_three_compound_assignment_and_trace)
{
    auto identity = math::SquareMatrix<float, 3>::Identity();

    auto sum = square3;
    sum += identity;
    EXPECT_NEAR(sum.at(0, 0), 2.0f, math::Tolerance<float>());

    auto difference = square3;
    difference -= identity;
    EXPECT_NEAR(difference.at(1, 1), 4.0f, math::Tolerance<float>());

    auto scaled = square3;
    scaled *= 2.0f;
    EXPECT_NEAR(scaled.at(2, 2), 20.0f, math::Tolerance<float>());

    EXPECT_NEAR(square3.Trace(), 16.0f, math::Tolerance<float>());
    EXPECT_NEAR(square3.Transpose().at(0, 2), 7.0f, math::Tolerance<float>());
}

TEST_F(MatrixFloatShapeTest, non_square_shapes_transpose_and_index)
{
    math::Matrix<float, 1, 4> row{ { 1.0f, 2.0f, 3.0f, 4.0f } };
    auto column = row.Transpose();

    EXPECT_NEAR(column.at(3, 0), 4.0f, math::Tolerance<float>());

    math::Matrix<float, 4, 3> tall{};
    tall.at(3, 2) = 5.0f;
    EXPECT_NEAR(tall.at(3, 2), 5.0f, math::Tolerance<float>());

    math::Matrix<float, 8, 2> block{};
    block.at(7, 1) = 2.0f;
    EXPECT_NEAR(block.at(7, 1), 2.0f, math::Tolerance<float>());
}

TEST_F(MatrixFloatShapeTest, large_vector_shapes_are_addressable)
{
    math::Vector<float, 30> long30{};
    long30.at(29, 0) = 3.0f;
    EXPECT_NEAR(long30.at(29, 0), 3.0f, math::Tolerance<float>());

    math::Vector<float, 128> long128{};
    long128.at(127, 0) = 7.0f;
    std::size_t count = 0;
    for (auto it = long128.begin(); it != long128.end(); ++it)
        ++count;
    EXPECT_EQ(count, 128u);
    EXPECT_NEAR(long128[127], 7.0f, math::Tolerance<float>());

    math::Vector<float, 5> five{};
    five.at(4, 0) = 1.0f;
    EXPECT_NEAR(five[4], 1.0f, math::Tolerance<float>());
}

TEST_F(MatrixFloatShapeTest, swap_rows_covers_vector_and_square_shapes)
{
    math::Vector<float, 30> column{};
    column.at(0, 0) = 1.0f;
    column.at(29, 0) = 2.0f;
    math::SwapRows(column, 0u, 29u);
    EXPECT_NEAR(column.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(column.at(29, 0), 1.0f, math::Tolerance<float>());

    math::SquareMatrix<float, 30> wide{};
    wide.at(0, 5) = 4.0f;
    math::SwapRows(wide, 0u, 1u);
    EXPECT_NEAR(wide.at(1, 5), 4.0f, math::Tolerance<float>());

    math::Matrix<float, 1, 1> single{};
    single.at(0, 0) = 9.0f;
    math::SwapRows(single, 0u, 0u);
    EXPECT_NEAR(single.at(0, 0), 9.0f, math::Tolerance<float>());
}
