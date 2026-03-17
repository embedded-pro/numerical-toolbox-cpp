#include "numerical/math/QNumber.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestDiscreteAlgebraicRiccatiEquation : public ::testing::Test
    {
    protected:
        solvers::DiscreteAlgebraicRiccatiEquation<float, 1, 1> scalarSolver;
        solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> vectorSolver;
    };
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solve_converges_immediately_when_dynamics_are_zero)
{
    math::SquareMatrix<float, 1> A{ { 0.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 2.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = scalarSolver.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 0), 2.0f, 1e-3f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solve_iterates_riccati_update)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = scalarSolver.Solve(A, B, Q, R);

    EXPECT_GT(P.at(0, 0), Q.at(0, 0));
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solve_produces_symmetric_result)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    auto Q = math::SquareMatrix<float, 2>::Identity();
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = vectorSolver.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), 1e-3f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solve_produces_positive_diagonal)
{
    math::SquareMatrix<float, 2> A{
        { 0.5f, 0.0f },
        { 0.0f, 0.3f }
    };

    math::Matrix<float, 2, 1> B{
        { 1.0f },
        { 0.0f }
    };

    auto Q = math::SquareMatrix<float, 2>::Identity();
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = vectorSolver.Solve(A, B, Q, R);

    EXPECT_GT(P.at(0, 0), 0.0f);
    EXPECT_GT(P.at(1, 1), 0.0f);
}
