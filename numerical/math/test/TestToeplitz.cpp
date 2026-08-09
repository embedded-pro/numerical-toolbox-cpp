#include "numerical/math/QNumber.hpp"
#include "numerical/math/Toeplitz.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>

namespace
{
    using math::test::AreMatricesNear;
    using math::test::AreVectorsNear;

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
            return T(std::max(std::min(f, 0.09f), -0.09f));
        }

        VectorType MakeVector(float a, float b)
        {
            return VectorType{
                { MakeValue(a) },
                { MakeValue(b) }
            };
        }

        MatrixType MakeMatrix(float a00, float a01, float a10, float a11)
        {
            return MatrixType{
                { MakeValue(a00), MakeValue(a01) },
                { MakeValue(a10), MakeValue(a11) }
            };
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(ToeplitzMatrixTest, TestTypes);
}

TYPED_TEST(ToeplitzMatrixTest, DefaultConstructorProducesAllZeroEntries)
{
    typename TestFixture::ToeplitzType t;

    auto full = t.ToFullMatrix();

    for (size_t i = 0; i < TestFixture::N; ++i)
        for (size_t j = 0; j < TestFixture::N; ++j)
            EXPECT_NEAR(math::ToFloat(full.at(i, j)), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, SymmetricConstructorProducesSymmetricToeplitzStructure)
{
    auto vec = this->MakeVector(0.06f, 0.03f);

    typename TestFixture::ToeplitzType t(vec);

    EXPECT_TRUE(t.IsSymmetric());
    EXPECT_TRUE(AreMatricesNear(t.ToFullMatrix(),
        this->MakeMatrix(
            0.06f, 0.03f,
            0.03f, 0.06f)));
}

TYPED_TEST(ToeplitzMatrixTest, GeneralConstructorProducesAsymmetricToeplitzStructure)
{
    auto row = this->MakeVector(0.06f, 0.03f);
    auto col = this->MakeVector(0.06f, -0.03f);

    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_FALSE(t.IsSymmetric());
    EXPECT_TRUE(AreMatricesNear(t.ToFullMatrix(),
        this->MakeMatrix(
            0.06f,  0.03f,
           -0.03f,  0.06f)));
}

TYPED_TEST(ToeplitzMatrixTest, ElementAccessMatchesConstructedRowAndColumn)
{
    auto row = this->MakeVector(0.06f, 0.03f);
    auto col = this->MakeVector(0.06f, -0.03f);
    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_NEAR(math::ToFloat(t.at(0, 0)),  0.06f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(t.at(0, 1)),  0.03f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(t.at(1, 0)), -0.03f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(t.at(1, 1)),  0.06f, math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, ToFullMatrixConsistentWithAtAccessor)
{
    auto row = this->MakeVector(0.06f, 0.03f);
    auto col = this->MakeVector(0.06f, -0.03f);
    typename TestFixture::ToeplitzType t(row, col);

    auto full = t.ToFullMatrix();

    for (size_t i = 0; i < TestFixture::N; ++i)
        for (size_t j = 0; j < TestFixture::N; ++j)
            EXPECT_NEAR(math::ToFloat(full.at(i, j)), math::ToFloat(t.at(i, j)), math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, VectorMultiplicationMatchesFullMatrixMultiply)
{
    auto vec = this->MakeVector(0.06f, 0.03f);
    typename TestFixture::ToeplitzType t(vec);
    auto x = this->MakeVector(0.01f, 0.02f);

    auto result = t * x;

    auto full = t.ToFullMatrix();
    typename TestFixture::VectorType expected;
    for (size_t i = 0; i < TestFixture::N; ++i)
    {
        float sum = 0.0f;
        for (size_t j = 0; j < TestFixture::N; ++j)
            sum += math::ToFloat(full.at(i, j)) * math::ToFloat(x.at(j, 0));
        expected.at(i, 0) = typename TestFixture::VectorType::value_type(sum);
    }

    for (size_t i = 0; i < TestFixture::N; ++i)
        EXPECT_NEAR(math::ToFloat(result.at(i, 0)), math::ToFloat(expected.at(i, 0)), math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, ZeroVectorMultiplicationProducesZeroVector)
{
    auto vec = this->MakeVector(0.06f, 0.03f);
    typename TestFixture::ToeplitzType t(vec);
    typename TestFixture::VectorType zero;

    auto result = t * zero;

    for (size_t i = 0; i < TestFixture::N; ++i)
        EXPECT_NEAR(math::ToFloat(result.at(i, 0)), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, AdditionProducesCorrectToeplitzSum)
{
    typename TestFixture::ToeplitzType t1(this->MakeVector(0.04f, 0.02f));
    typename TestFixture::ToeplitzType t2(this->MakeVector(0.02f, 0.01f));

    auto result = t1 + t2;

    EXPECT_TRUE(AreMatricesNear(result.ToFullMatrix(),
        this->MakeMatrix(
            0.06f, 0.03f,
            0.03f, 0.06f)));
}

TYPED_TEST(ToeplitzMatrixTest, AdditionIsCommutative)
{
    typename TestFixture::ToeplitzType t1(this->MakeVector(0.04f, 0.02f));
    typename TestFixture::ToeplitzType t2(this->MakeVector(0.02f, 0.01f));

    auto ab = t1 + t2;
    auto ba = t2 + t1;

    EXPECT_TRUE(AreMatricesNear(ab.ToFullMatrix(), ba.ToFullMatrix()));
}

TYPED_TEST(ToeplitzMatrixTest, SubtractionProducesCorrectToeplitzDifference)
{
    typename TestFixture::ToeplitzType t1(this->MakeVector(0.06f, 0.03f));
    typename TestFixture::ToeplitzType t2(this->MakeVector(0.02f, 0.01f));

    auto result = t1 - t2;

    EXPECT_TRUE(AreMatricesNear(result.ToFullMatrix(),
        this->MakeMatrix(
            0.04f, 0.02f,
            0.02f, 0.04f)));
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrixAcceptsValidPattern)
{
    auto matrix = this->MakeMatrix(
        0.06f, 0.03f,
        0.03f, 0.06f);

    EXPECT_TRUE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrixRejectsNonToeplitzPattern)
{
    auto matrix = this->MakeMatrix(
        0.06f, 0.03f,
        0.03f, 0.09f);

    EXPECT_FALSE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, IsToeplitzMatrixAcceptsZeroMatrix)
{
    auto matrix = this->MakeMatrix(
        0.0f, 0.0f,
        0.0f, 0.0f);

    EXPECT_TRUE(TestFixture::ToeplitzType::IsToeplitzMatrix(matrix));
}

TYPED_TEST(ToeplitzMatrixTest, ToFullMatrixPassesIsToeplitzCheck)
{
    auto row = this->MakeVector(0.06f, 0.03f);
    auto col = this->MakeVector(0.06f, -0.03f);
    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_TRUE(TestFixture::ToeplitzType::IsToeplitzMatrix(t.ToFullMatrix()));
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectorsRecoverRowAndColumn)
{
    auto matrix = this->MakeMatrix(
        0.06f, 0.03f,
        0.03f, 0.06f);

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    EXPECT_NEAR(math::ToFloat(row.at(0, 0)), 0.06f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(row.at(1, 0)), 0.03f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(col.at(0, 0)), 0.06f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(col.at(1, 0)), 0.03f, math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, ExtractThenConstructRoundTripMatchesOriginalMatrix)
{
    auto matrix = this->MakeMatrix(
        0.06f, 0.03f,
       -0.03f, 0.06f);

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);
    typename TestFixture::ToeplitzType t(row, col);

    EXPECT_TRUE(AreMatricesNear(t.ToFullMatrix(), matrix));
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectorsSymmetricMatrixYieldsEqualRowAndColumn)
{
    auto matrix = this->MakeMatrix(
        0.06f, 0.03f,
        0.03f, 0.06f);

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    for (size_t i = 0; i < TestFixture::N; ++i)
        EXPECT_NEAR(math::ToFloat(row.at(i, 0)), math::ToFloat(col.at(i, 0)), math::Tolerance<float>());
}

TYPED_TEST(ToeplitzMatrixTest, ExtractToeplitzVectorsZeroMatrixYieldsZeroVectors)
{
    auto matrix = this->MakeMatrix(
        0.0f, 0.0f,
        0.0f, 0.0f);

    auto [row, col] = TestFixture::ToeplitzType::ExtractToeplitzVectors(matrix);

    for (size_t i = 0; i < TestFixture::N; ++i)
    {
        EXPECT_NEAR(math::ToFloat(row.at(i, 0)), 0.0f, math::Tolerance<float>());
        EXPECT_NEAR(math::ToFloat(col.at(i, 0)), 0.0f, math::Tolerance<float>());
    }
}

TYPED_TEST(ToeplitzMatrixTest, CreateToeplitzMatrixFactoryProducesSameResultAsConstructor)
{
    auto vec = this->MakeVector(0.06f, 0.03f);

    auto fromFactory = math::CreateToeplitzMatrix(vec);
    typename TestFixture::ToeplitzType fromCtor(vec);

    EXPECT_TRUE(AreMatricesNear(fromFactory.ToFullMatrix(), fromCtor.ToFullMatrix()));
}

TYPED_TEST(ToeplitzMatrixTest, TwoInstancesWithSameInputProduceIdenticalOutput)
{
    auto row = this->MakeVector(0.06f, 0.03f);
    auto col = this->MakeVector(0.06f, -0.03f);

    typename TestFixture::ToeplitzType t1(row, col);
    typename TestFixture::ToeplitzType t2(row, col);

    auto x = this->MakeVector(0.02f, 0.04f);
    auto r1 = t1 * x;
    auto r2 = t2 * x;

    for (size_t i = 0; i < TestFixture::N; ++i)
        EXPECT_FLOAT_EQ(math::ToFloat(r1.at(i, 0)), math::ToFloat(r2.at(i, 0)));
}
