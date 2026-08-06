#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestDiscreteAlgebraicRiccatiEquation : public ::testing::Test
    {
    protected:
        solvers::DiscreteAlgebraicRiccatiEquation<float, 1, 1> scalarSolver;
        solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> twoByOneSolver;
        solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 2> twoByTwoSolver;

        static float DareResidual2x1(
            const math::SquareMatrix<float, 2>& P,
            const math::SquareMatrix<float, 2>& A,
            const math::Matrix<float, 2, 1>& B,
            const math::SquareMatrix<float, 2>& Q,
            const math::SquareMatrix<float, 1>& R)
        {
            auto BtP = B.Transpose() * P;
            auto S = R + BtP * B;
            auto AtP = A.Transpose() * P;
            auto AtPA = AtP * A;
            auto BtPA = BtP * A;
            auto correction = AtP * B * (math::Matrix<float, 1, 2>{ { BtPA.at(0, 0), BtPA.at(0, 1) } } * (1.0f / S.at(0, 0)));
            auto Prhs = AtPA - correction + Q;

            float maxErr = 0.0f;
            for (std::size_t i = 0; i < 2; ++i)
                for (std::size_t j = 0; j < 2; ++j)
                    maxErr = std::max(maxErr, std::abs(Prhs.at(i, j) - P.at(i, j)));
            return maxErr;
        }
    };
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, scalar_unit_system_matches_golden_ratio_root)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = scalarSolver.Solve(A, B, Q, R);

    constexpr float expected = 1.6180339887f;
    EXPECT_NEAR(P.at(0, 0), expected, 1e-3f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, scalar_stable_system_matches_closed_form_quadratic_root)
{
    math::SquareMatrix<float, 1> A{ { 0.9f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = scalarSolver.Solve(A, B, Q, R);

    constexpr float expected = 1.483901f;
    EXPECT_NEAR(P.at(0, 0), expected, 1e-3f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, zero_dynamics_converges_immediately_to_Q)
{
    math::SquareMatrix<float, 1> A{ { 0.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 2.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = scalarSolver.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 0), 2.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, two_by_one_solution_satisfies_dare_residual)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };
    auto Q = math::SquareMatrix<float, 2>::Identity();
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = twoByOneSolver.Solve(A, B, Q, R);

    float residual = DareResidual2x1(P, A, B, Q, R);
    EXPECT_NEAR(residual, 0.0f, 1e-2f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, two_by_one_solution_is_symmetric)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };
    auto Q = math::SquareMatrix<float, 2>::Identity();
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = twoByOneSolver.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, two_by_one_solution_is_positive_definite)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };
    auto Q = math::SquareMatrix<float, 2>::Identity();
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    auto P = twoByOneSolver.Solve(A, B, Q, R);

    float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
    EXPECT_GT(P.at(0, 0), 0.0f);
    EXPECT_GT(det, 0.0f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, two_by_two_full_input_solution_is_symmetric_and_positive_definite)
{
    math::SquareMatrix<float, 2> A{
        { 0.8f, 0.2f },
        { 0.0f, 0.7f }
    };
    math::Matrix<float, 2, 2> B{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    auto Q = math::SquareMatrix<float, 2>::Identity();
    auto R = math::SquareMatrix<float, 2>::Identity();

    auto P = twoByTwoSolver.Solve(A, B, Q, R);

    float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
    EXPECT_GT(P.at(0, 0), 0.0f);
    EXPECT_GT(det, 0.0f);
}
