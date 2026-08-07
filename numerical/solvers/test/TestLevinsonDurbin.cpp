#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "numerical/solvers/LevinsonDurbin.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestLevinsonDurbin
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 2;
        solvers::LevinsonDurbin<float, N> solver;
    };

    class TestLevinsonDurbin3
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 3;
        solvers::LevinsonDurbin<float, N> solver;
        solvers::GaussianElimination<float, N> reference;

        math::Matrix<float, N, N> BuildToeplitz3(float r0, float r1, float r2)
        {
            math::Vector<float, N> row{ { r0 }, { r1 }, { r2 } };
            return math::ToeplitzMatrix<float, N>(row).ToFullMatrix();
        }
    };
}

TEST_F(TestLevinsonDurbin, solve_symmetric_toeplitz_returns_correct_coefficients)
{
    math::Vector<float, 2> r{ { 0.02f }, { 0.01f } };
    auto toeplitz = math::ToeplitzMatrix<float, 2>(r);
    auto A = toeplitz.ToFullMatrix();
    math::Vector<float, 2> b{ { 0.01f }, { 0.005f } };

    auto x = solver.Solve(A, b);

    EXPECT_NEAR(x.at(0, 0), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestLevinsonDurbin, solve_asserts_on_non_toeplitz_matrix)
{
    math::Matrix<float, 2, 2> A{
        { 0.01f, 0.02f },
        { 0.03f, 0.04f }
    };
    math::Vector<float, 2> b{ { 0.01f }, { 0.005f } };

    EXPECT_DEATH_IF_SUPPORTED(solver.Solve(A, b), "");
}

TEST_F(TestLevinsonDurbin3, solve_n3_doc_reference_example)
{
    auto A = BuildToeplitz3(4.0f, 2.0f, 1.0f);
    math::Vector<float, 3> b{ { 2.0f }, { 1.0f }, { 0.5f } };

    auto x = solver.Solve(A, b);

    EXPECT_NEAR(x.at(0, 0), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(2, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestLevinsonDurbin3, solve_n3_residual_satisfies_axb)
{
    auto A = BuildToeplitz3(4.0f, 2.0f, 1.0f);
    math::Vector<float, 3> b{ { 2.0f }, { 1.0f }, { 0.5f } };

    auto x = solver.Solve(A, b);

    math::Vector<float, 3> row{ { 4.0f }, { 2.0f }, { 1.0f } };
    auto toeplitz = math::ToeplitzMatrix<float, 3>(row);
    auto Ax = toeplitz * x;

    EXPECT_NEAR(Ax.at(0, 0), b.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(Ax.at(1, 0), b.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(Ax.at(2, 0), b.at(2, 0), math::Tolerance<float>());
}

TEST_F(TestLevinsonDurbin3, solve_n3_matches_gaussian_elimination)
{
    auto A = BuildToeplitz3(6.0f, 3.0f, 1.0f);
    math::Vector<float, 3> b{ { 1.0f }, { 2.0f }, { 3.0f } };

    auto xLd = solver.Solve(A, b);
    auto xGe = reference.Solve(A, b);

    EXPECT_NEAR(xLd.at(0, 0), xGe.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(xLd.at(1, 0), xGe.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(xLd.at(2, 0), xGe.at(2, 0), math::Tolerance<float>());
}

TEST_F(TestLevinsonDurbin3, solve_n3_reflection_coefficients_bounded_for_spd_matrix)
{
    auto A = BuildToeplitz3(4.0f, 2.0f, 1.0f);
    math::Vector<float, 3> b{ { 2.0f }, { 1.0f }, { 0.5f } };

    auto x = solver.Solve(A, b);

    float mu0 = b.at(0, 0) / 4.0f;
    float residual0 = 4.0f * (1.0f - mu0 * mu0);
    float alpha1 = b.at(1, 0) - x.at(0, 0) * 2.0f;
    float mu1 = alpha1 / residual0;

    EXPECT_LT(std::abs(mu0), 1.0f);
    EXPECT_LT(std::abs(mu1), 1.0f);
}

TEST_F(TestLevinsonDurbin3, solve_n3_deterministic_on_repeated_calls)
{
    auto A = BuildToeplitz3(4.0f, 2.0f, 1.0f);
    math::Vector<float, 3> b{ { 2.0f }, { 1.0f }, { 0.5f } };

    auto x1 = solver.Solve(A, b);
    auto x2 = solver.Solve(A, b);

    EXPECT_FLOAT_EQ(x1.at(0, 0), x2.at(0, 0));
    EXPECT_FLOAT_EQ(x1.at(1, 0), x2.at(1, 0));
    EXPECT_FLOAT_EQ(x1.at(2, 0), x2.at(2, 0));
}
