#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/LyapunovSylvester.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestSylvesterSolver : public ::testing::Test
    {
    protected:
        solvers::SylvesterSolver<float, 2, 2> sylv{};
        solvers::SylvesterSolver<float, 3, 3> lyap3{};
    };
}

TEST_F(TestSylvesterSolver, sylvester_residual_AX_plus_XB_equals_C)
{
    math::SquareMatrix<float, 2> A{ { -3.0f, 1.0f }, { 0.0f, -2.0f } };
    math::SquareMatrix<float, 2> B{ { 1.0f, 2.0f }, { 0.0f, 1.0f } };
    math::SquareMatrix<float, 2> C{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveSylvester(A, B, C);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    auto residual = A * X + X * B;
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(residual.at(i, j), C.at(i, j), math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, sylvester_unsolvable_pair_returns_false)
{
    math::SquareMatrix<float, 2> A{ { 1.0f, 0.0f }, { 0.0f, 2.0f } };
    math::SquareMatrix<float, 2> B{ { -1.0f, 0.0f }, { 0.0f, -2.0f } };
    math::SquareMatrix<float, 2> C{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveSylvester(A, B, C);

    EXPECT_FALSE(ok);
}

TEST_F(TestSylvesterSolver, sylvester_scalar_closed_form)
{
    solvers::SylvesterSolver<float, 1, 1> sylv1{};
    math::SquareMatrix<float, 1> a{ { 3.0f } };
    math::SquareMatrix<float, 1> c{ { 6.0f } };

    bool ok = sylv1.SolveSylvester(a, a, c);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(sylv1.Solution().at(0, 0), 1.0f, math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, continuous_lyapunov_spd_and_residual_for_stable_A)
{
    math::SquareMatrix<float, 2> A{ { -2.0f, 1.0f }, { 0.0f, -3.0f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveContinuousLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    EXPECT_GT(X.at(0, 0), 0.0f);
    EXPECT_GT(X.at(1, 1), 0.0f);
    EXPECT_GT(X.at(0, 0) * X.at(1, 1) - X.at(0, 1) * X.at(1, 0), 0.0f);
    EXPECT_NEAR(X.at(0, 1), X.at(1, 0), math::Tolerance<float>());
    auto residual = A * X + X * A.Transpose();
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(residual.at(i, j), -Q.at(i, j), math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, continuous_lyapunov_diagonal_A_matches_closed_form)
{
    math::SquareMatrix<float, 2> A{ { -2.0f, 0.0f }, { 0.0f, -3.0f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveContinuousLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    EXPECT_NEAR(X.at(0, 0), 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(X.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(X.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(X.at(1, 1), 1.0f / 6.0f, math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, continuous_lyapunov_unstable_A_returns_false)
{
    math::SquareMatrix<float, 2> A{ { 1.0f, 0.0f }, { 0.0f, -1.0f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveContinuousLyapunov(A, Q);

    EXPECT_FALSE(ok);
}

TEST_F(TestSylvesterSolver, continuous_lyapunov_symmetric_solution_for_diagonal_A)
{
    math::SquareMatrix<float, 3> A{ { -1.0f, 0.0f, 0.0f }, { 0.0f, -2.0f, 0.0f }, { 0.0f, 0.0f, -3.0f } };
    math::SquareMatrix<float, 3> Q{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

    bool ok = lyap3.SolveContinuousLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = lyap3.Solution();
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(X.at(i, j), X.at(j, i), math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, discrete_lyapunov_residual_AXAt_minus_X_equals_minus_Q)
{
    math::SquareMatrix<float, 2> A{ { 0.5f, 0.0f }, { 0.0f, 0.3f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveDiscreteLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    auto residual = A * X * A.Transpose() - X;
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(residual.at(i, j), -Q.at(i, j), math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, discrete_lyapunov_diagonal_A_matches_closed_form)
{
    math::SquareMatrix<float, 2> A{ { 0.5f, 0.0f }, { 0.0f, 0.3f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveDiscreteLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    EXPECT_NEAR(X.at(0, 0), 1.0f / (1.0f - 0.5f * 0.5f), math::Tolerance<float>());
    EXPECT_NEAR(X.at(1, 1), 1.0f / (1.0f - 0.3f * 0.3f), math::Tolerance<float>());
    EXPECT_NEAR(X.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(X.at(1, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, discrete_lyapunov_symmetric_solution_for_symmetric_Q)
{
    math::SquareMatrix<float, 2> A{ { 0.5f, 0.1f }, { 0.0f, 0.3f } };
    math::SquareMatrix<float, 2> Q{ { 2.0f, 0.5f }, { 0.5f, 3.0f } };

    bool ok = sylv.SolveDiscreteLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(X.at(i, j), X.at(j, i), math::Tolerance<float>());
}

TEST_F(TestSylvesterSolver, discrete_lyapunov_gramian_matches_truncated_series)
{
    math::SquareMatrix<float, 2> A{ { 0.5f, 0.0f }, { 0.0f, 0.3f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 1.0f }, { 1.0f, 1.0f } };

    bool ok = sylv.SolveDiscreteLyapunov(A, Q);

    EXPECT_TRUE(ok);
    auto X = sylv.Solution();

    math::SquareMatrix<float, 2> truncSum{};
    math::SquareMatrix<float, 2> Ak = math::SquareMatrix<float, 2>::Identity();
    for (std::size_t k = 0; k <= 40; ++k)
    {
        truncSum += Ak * Q * Ak.Transpose();
        Ak = A * Ak;
    }

    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_NEAR(X.at(i, j), truncSum.at(i, j), 1e-3f);
}

TEST_F(TestSylvesterSolver, discrete_lyapunov_unstable_A_returns_false)
{
    math::SquareMatrix<float, 2> A{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    bool ok = sylv.SolveDiscreteLyapunov(A, Q);

    EXPECT_FALSE(ok);
}

TEST_F(TestSylvesterSolver, solution_is_zero_before_any_solve)
{
    solvers::SylvesterSolver<float, 2, 2> fresh{};
    auto X = fresh.Solution();
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_FLOAT_EQ(X.at(i, j), 0.0f);
}

TEST_F(TestSylvesterSolver, determinism_same_input_gives_identical_output)
{
    math::SquareMatrix<float, 2> A{ { -2.0f, 0.5f }, { 0.0f, -3.0f } };
    math::SquareMatrix<float, 2> Q{ { 1.0f, 0.0f }, { 0.0f, 1.0f } };

    solvers::SylvesterSolver<float, 2, 2> s1{};
    solvers::SylvesterSolver<float, 2, 2> s2{};
    s1.SolveContinuousLyapunov(A, Q);
    s2.SolveContinuousLyapunov(A, Q);

    auto X1 = s1.Solution();
    auto X2 = s2.Solution();
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
            EXPECT_FLOAT_EQ(X1.at(i, j), X2.at(i, j));
}
