#include "numerical/math/Tolerance.hpp"
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

    EXPECT_DEATH(solver.Solve(A, b), "");
}
