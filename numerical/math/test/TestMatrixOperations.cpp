#include "numerical/math/MatrixOperations.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class MatrixOperationsTest : public ::testing::Test
    {
    protected:
        math::SquareMatrix<float, 2> asymmetric{
            { 1.0f, 3.0f },
            { -1.0f, 2.0f }
        };
    };
}

TEST_F(MatrixOperationsTest, SymmetrizeAveragesOffDiagonals)
{
    auto s = math::Symmetrize(asymmetric);

    EXPECT_NEAR(s.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(s.at(1, 1), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(s.at(0, 1), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(s.at(1, 0), 1.0f, math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, SymmetrizeIsIdempotentOnSymmetricInput)
{
    auto once = math::Symmetrize(asymmetric);
    auto twice = math::Symmetrize(once);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(twice.at(i, j), once.at(i, j), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, CongruenceTransformMatchesExplicitProduct)
{
    math::Matrix<float, 2, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f }
    };
    math::SquareMatrix<float, 3> m{
        { 2.0f, 0.0f, 1.0f },
        { 0.0f, 3.0f, 0.0f },
        { 1.0f, 0.0f, 4.0f }
    };

    auto result = math::CongruenceTransform(a, m);
    auto expected = a * m * a.Transpose();

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(result.at(i, j), expected.at(i, j), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, CongruenceTransformPreservesSymmetry)
{
    math::Matrix<float, 2, 2> a{
        { 1.0f, 2.0f },
        { -3.0f, 0.5f }
    };
    math::SquareMatrix<float, 2> symmetric{
        { 4.0f, 1.0f },
        { 1.0f, 2.0f }
    };

    auto result = math::CongruenceTransform(a, symmetric);

    EXPECT_NEAR(result.at(0, 1), result.at(1, 0), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, SymmetrizeZeroMatrixYieldsZero)
{
    math::SquareMatrix<float, 2> zero{
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };

    auto result = math::Symmetrize(zero);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(result.at(i, j), 0.0f, math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, SymmetrizeOutputSatisfiesSymmetryInvariant)
{
    auto result = math::Symmetrize(asymmetric);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(result.at(i, j), result.at(j, i), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, SymmetrizeAlreadySymmetricMatrixIsUnchanged)
{
    math::SquareMatrix<float, 2> sym{
        { 3.0f, -1.0f },
        { -1.0f, 5.0f }
    };

    auto result = math::Symmetrize(sym);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(result.at(i, j), sym.at(i, j), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, CongruenceTransformWithIdentityEqualsAAt)
{
    math::Matrix<float, 2, 2> a{
        { 1.0f, 2.0f },
        { 3.0f, 4.0f }
    };
    auto identity = math::SquareMatrix<float, 2>::Identity();

    auto result = math::CongruenceTransform(a, identity);

    EXPECT_NEAR(result.at(0, 0), 5.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 11.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 11.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), 25.0f, math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, CongruenceTransformScalesLinearlyWithScalarMatrix)
{
    math::Matrix<float, 2, 2> a{
        { 1.0f, 2.0f },
        { 3.0f, 4.0f }
    };
    auto identity = math::SquareMatrix<float, 2>::Identity();
    const float scalar = 3.0f;
    math::SquareMatrix<float, 2> scaledIdentity{
        { scalar, 0.0f },
        { 0.0f, scalar }
    };

    auto base = math::CongruenceTransform(a, identity);
    auto scaled = math::CongruenceTransform(a, scaledIdentity);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(scaled.at(i, j), scalar * base.at(i, j), math::Tolerance<float>());
}

TEST_F(MatrixOperationsTest, CongruenceTransformZeroMYieldsZeroResult)
{
    math::Matrix<float, 2, 2> a{
        { 1.0f, 2.0f },
        { 3.0f, 4.0f }
    };
    math::SquareMatrix<float, 2> zeroM{
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };

    auto result = math::CongruenceTransform(a, zeroM);

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(result.at(i, j), 0.0f, math::Tolerance<float>());
}
