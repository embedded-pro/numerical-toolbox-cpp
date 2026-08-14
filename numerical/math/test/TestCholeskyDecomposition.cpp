#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class CholeskyDecompositionTest : public ::testing::Test
    {
    protected:
        using Chol1 = math::CholeskyDecomposition<float, 1>;
        using Chol2 = math::CholeskyDecomposition<float, 2>;
        using Mat1 = math::SquareMatrix<float, 1>;
        using Mat2 = math::SquareMatrix<float, 2>;
        using Vec2 = math::Vector<float, 2>;
    };
}

TEST_F(CholeskyDecompositionTest, TryFactorScalarSpdMatrixSucceeds)
{
    Mat1 a;
    a.at(0, 0) = 4.0f;

    auto l = Chol1::TryFactor(a);

    ASSERT_TRUE(l.has_value());
    EXPECT_NEAR(l->at(0, 0), 2.0f, math::Tolerance<float>());
}

TEST_F(CholeskyDecompositionTest, TryFactorSingularScalarReturnsNullopt)
{
    Mat1 a;
    a.at(0, 0) = 0.0f;

    EXPECT_FALSE(Chol1::TryFactor(a).has_value());
}

TEST_F(CholeskyDecompositionTest, TryFactor2x2SpdMatrixCoversOffDiagonalBranch)
{
    Mat2 a;
    a.at(0, 0) = 4.0f; a.at(0, 1) = 2.0f;
    a.at(1, 0) = 2.0f; a.at(1, 1) = 3.0f;

    auto l = Chol2::TryFactor(a);

    ASSERT_TRUE(l.has_value());
    EXPECT_NEAR(l->at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(l->at(1, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(l->at(1, 1), std::sqrt(2.0f), math::Tolerance<float>());
}

TEST_F(CholeskyDecompositionTest, TryFactorNonSpdMatrixReturnsNullopt)
{
    Mat2 a;
    a.at(0, 0) = 1.0f; a.at(0, 1) = 2.0f;
    a.at(1, 0) = 2.0f; a.at(1, 1) = 1.0f;

    EXPECT_FALSE(Chol2::TryFactor(a).has_value());
}

TEST_F(CholeskyDecompositionTest, TryFactorSmallScaleSpdMatrixAccepted)
{
    Mat1 a;
    a.at(0, 0) = 5e-11f;

    auto l = Chol1::TryFactor(a);

    ASSERT_TRUE(l.has_value());
    EXPECT_GT(l->at(0, 0), 0.0f);
}

TEST_F(CholeskyDecompositionTest, FactorNonSpdReturnsZeroMatrix)
{
    Mat2 a;
    a.at(0, 0) = 1.0f; a.at(0, 1) = 2.0f;
    a.at(1, 0) = 2.0f; a.at(1, 1) = 1.0f;

    auto l = Chol2::Factor(a);

    EXPECT_NEAR(l.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(l.at(1, 1), 0.0f, math::Tolerance<float>());
}

TEST_F(CholeskyDecompositionTest, SolveDiagonalSystemFindsExactSolution)
{
    Mat2 a;
    a.at(0, 0) = 4.0f; a.at(0, 1) = 0.0f;
    a.at(1, 0) = 0.0f; a.at(1, 1) = 9.0f;
    Vec2 b;
    b.at(0, 0) = 8.0f;
    b.at(1, 0) = 27.0f;

    auto x = Chol2::Solve(a, b);

    ASSERT_TRUE(x.has_value());
    EXPECT_NEAR(x->at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(x->at(1, 0), 3.0f, math::Tolerance<float>());
}

TEST_F(CholeskyDecompositionTest, SolveNonSpdSystemReturnsNullopt)
{
    Mat2 a;
    a.at(0, 0) = 1.0f; a.at(0, 1) = 2.0f;
    a.at(1, 0) = 2.0f; a.at(1, 1) = 1.0f;
    Vec2 b;
    b.at(0, 0) = 1.0f;
    b.at(1, 0) = 1.0f;

    EXPECT_FALSE(Chol2::Solve(a, b).has_value());
}
