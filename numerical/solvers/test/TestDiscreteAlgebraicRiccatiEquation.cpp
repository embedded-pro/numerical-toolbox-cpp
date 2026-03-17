#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestDiscreteAlgebraicRiccatiEquation : public ::testing::Test
    {
    protected:
        static bool AreNear(float a, float b, float epsilon = 1e-3f)
        {
            return std::abs(a - b) < epsilon;
        }

        template<std::size_t N>
        static bool AreMatricesNear(const math::SquareMatrix<float, N>& a,
            const math::SquareMatrix<float, N>& b, float epsilon = 1e-3f)
        {
            for (std::size_t i = 0; i < N; ++i)
                for (std::size_t j = 0; j < N; ++j)
                    if (std::abs(a.at(i, j) - b.at(i, j)) > epsilon)
                        return false;

            return true;
        }
    };
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solves_1x1_scalar_system)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    solvers::DiscreteAlgebraicRiccatiEquation<float, 1, 1> dare;
    auto P = dare.Solve(A, B, Q, R);

    float goldenRatio = (1.0f + std::sqrt(5.0f)) / 2.0f;
    EXPECT_NEAR(P.at(0, 0), goldenRatio, 1e-3f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, solves_2x2_double_integrator)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    math::SquareMatrix<float, 2> Q{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::SquareMatrix<float, 1> R{ { 1.0f } };

    solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> dare;
    auto P = dare.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), 1e-3f);

    auto BtP = B.Transpose() * P;
    auto S = R + BtP * B;
    auto BtPA = BtP * A;
    auto K_part = solvers::SolveSystem<float, 1, 2>(S, BtPA);
    auto AtP = A.Transpose() * P;
    auto Pcheck = AtP * A - AtP * B * K_part + Q;

    EXPECT_TRUE(AreMatricesNear<2>(P, Pcheck, 1e-2f));
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, converges_for_stable_system)
{
    math::SquareMatrix<float, 2> A{
        { 0.5f, 0.0f },
        { 0.0f, 0.3f }
    };

    math::Matrix<float, 2, 1> B{
        { 1.0f },
        { 0.0f }
    };

    math::SquareMatrix<float, 2> Q{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::SquareMatrix<float, 1> R{ { 1.0f } };

    solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> dare;
    auto P = dare.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), 1e-3f);

    EXPECT_GT(P.at(0, 0), 0.0f);
    EXPECT_GT(P.at(1, 1), 0.0f);
}

TEST_F(TestDiscreteAlgebraicRiccatiEquation, identity_cost_with_no_dynamics_returns_Q)
{
    math::SquareMatrix<float, 1> A{ { 0.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 2.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    solvers::DiscreteAlgebraicRiccatiEquation<float, 1, 1> dare;
    auto P = dare.Solve(A, B, Q, R);

    EXPECT_NEAR(P.at(0, 0), 2.0f, 1e-3f);
}
