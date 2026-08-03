#include "numerical/math/MatrixExponential.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestMatrixExponential : public ::testing::Test
    {
    protected:
        math::MatrixExponential<float, 1> expm1{};
        math::MatrixExponential<float, 2> expm2{};
        math::MatrixExponential<float, 3> expm3{};
    };
}

TEST_F(TestMatrixExponential, ZeroMatrixGivesIdentity)
{
    math::SquareMatrix<float, 2> a{};
    auto result = expm2.Compute(a);
    EXPECT_NEAR(result.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), 1.0f, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, DiagonalIsElementwiseExp)
{
    math::SquareMatrix<float, 2> a{
        { 1.0f, 0.0f },
        { 0.0f, -2.0f }
    };
    auto result = expm2.Compute(a);
    EXPECT_NEAR(result.at(0, 0), std::exp(1.0f), math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), std::exp(-2.0f), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, Scalar1x1MatchesExp)
{
    math::SquareMatrix<float, 1> a{};
    a.at(0, 0) = 2.5f;
    auto result = expm1.Compute(a);
    EXPECT_NEAR(result.at(0, 0), std::exp(2.5f), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, NilpotentIsPolynomial)
{
    math::SquareMatrix<float, 2> a{
        { 0.0f, 1.0f },
        { 0.0f, 0.0f }
    };
    auto result = expm2.Compute(a);
    EXPECT_NEAR(result.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), 1.0f, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, RotationGenerator)
{
    const float theta = 1.0f;
    math::SquareMatrix<float, 2> a{
        { 0.0f, -theta },
        { theta, 0.0f }
    };
    auto result = expm2.Compute(a);
    EXPECT_NEAR(result.at(0, 0), std::cos(theta), math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), -std::sin(theta), math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), std::sin(theta), math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), std::cos(theta), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, Known2x2Reference)
{
    math::SquareMatrix<float, 2> a{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    auto result = expm2.Compute(a);
    const float e = std::exp(1.0f);
    EXPECT_NEAR(result.at(0, 0), e, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), e, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), e, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, LargeNormUsesScaling)
{
    math::SquareMatrix<float, 2> a{
        { 10.0f, 0.0f },
        { 0.0f, -10.0f }
    };
    auto result = expm2.Compute(a);
    EXPECT_NEAR(result.at(0, 0), std::exp(10.0f), 1.0f);
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), std::exp(-10.0f), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, ExpZeroDtIsIdentity)
{
    math::SquareMatrix<float, 2> a{
        { 1.0f, 2.0f },
        { 3.0f, 4.0f }
    };
    auto result = expm2.Compute(a, 0.0f);
    EXPECT_NEAR(result.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), 1.0f, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, GroupLawExpAExpNegAIsIdentity)
{
    math::SquareMatrix<float, 2> a{
        { 1.0f, 2.0f },
        { 0.0f, -1.0f }
    };
    math::SquareMatrix<float, 2> neg{
        { -1.0f, -2.0f },
        { 0.0f, 1.0f }
    };
    auto expA = expm2.Compute(a);
    auto expNegA = expm2.Compute(neg);
    auto product = expA * expNegA;
    EXPECT_NEAR(product.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(product.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(product.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(product.at(1, 1), 1.0f, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, DeterminantIdentityJacobiFormula)
{
    math::SquareMatrix<float, 2> a{
        { 2.0f, 1.0f },
        { 0.0f, 3.0f }
    };
    auto result = expm2.Compute(a);
    const float det = result.at(0, 0) * result.at(1, 1) - result.at(0, 1) * result.at(1, 0);
    const float trace = a.at(0, 0) + a.at(1, 1);
    EXPECT_NEAR(det, std::exp(trace), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, ThreeByThreeDiagonalIsElementwiseExp)
{
    math::SquareMatrix<float, 3> a{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f },
        { 0.0f, 0.0f, 2.0f }
    };
    auto result = expm3.Compute(a);
    EXPECT_NEAR(result.at(0, 0), std::exp(1.0f), math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), std::exp(-1.0f), math::Tolerance<float>());
    EXPECT_NEAR(result.at(2, 2), std::exp(2.0f), math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 2), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 2), 0.0f, math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, ComputeWithDtScalesDiagonal)
{
    math::SquareMatrix<float, 2> a{
        { 2.0f, 0.0f },
        { 0.0f, -3.0f }
    };
    const float dt = 0.5f;
    auto result = expm2.Compute(a, dt);
    EXPECT_NEAR(result.at(0, 0), std::exp(2.0f * dt), math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), std::exp(-3.0f * dt), math::Tolerance<float>());
}

TEST_F(TestMatrixExponential, StiffNegativeEigenvalueNearZeroNoNaN)
{
    math::SquareMatrix<float, 2> a{
        { -50.0f, 0.0f },
        { 0.0f, -50.0f }
    };
    auto result = expm2.Compute(a);
    EXPECT_FALSE(std::isnan(result.at(0, 0)));
    EXPECT_FALSE(std::isnan(result.at(1, 1)));
    EXPECT_FALSE(std::isinf(result.at(0, 0)));
    EXPECT_FALSE(std::isinf(result.at(1, 1)));
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
}
