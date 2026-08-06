#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/JacobiEigenSolver.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestJacobiEigenSolver : public ::testing::Test
    {
    protected:
        solvers::JacobiEigenSolver<float, 2> solver2{};
        solvers::JacobiEigenSolver<float, 3> solver3{};
        solvers::JacobiEigenSolver<float, 4> solver4{};
    };
}

TEST_F(TestJacobiEigenSolver, diagonal_matrix_returns_sorted_diagonal)
{
    math::Matrix<float, 3, 3> a{
        { 3.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 2.0f }
    };

    EXPECT_TRUE(solver3.Solve(a));

    EXPECT_NEAR(solver3.Eigenvalues().at(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(solver3.Eigenvalues().at(1, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(solver3.Eigenvalues().at(2, 0), 3.0f, 1e-5f);
}

TEST_F(TestJacobiEigenSolver, symmetric_2x2_matches_closed_form)
{
    math::Matrix<float, 2, 2> a{
        { 2.0f, 1.0f },
        { 1.0f, 2.0f }
    };

    EXPECT_TRUE(solver2.Solve(a));

    EXPECT_NEAR(solver2.Eigenvalues().at(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(solver2.Eigenvalues().at(1, 0), 3.0f, 1e-5f);
}

TEST_F(TestJacobiEigenSolver, recovers_known_eigenvalues_of_dense_symmetric)
{
    math::Matrix<float, 3, 3> a{
        { 4.0f, 1.0f, 1.0f },
        { 1.0f, 4.0f, 1.0f },
        { 1.0f, 1.0f, 4.0f }
    };

    EXPECT_TRUE(solver3.Solve(a));

    EXPECT_NEAR(solver3.Eigenvalues().at(0, 0), 3.0f, 1e-5f);
    EXPECT_NEAR(solver3.Eigenvalues().at(1, 0), 3.0f, 1e-5f);
    EXPECT_NEAR(solver3.Eigenvalues().at(2, 0), 6.0f, 1e-5f);
}

TEST_F(TestJacobiEigenSolver, eigenvectors_are_orthonormal)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, -1.0f, 0.0f },
        { -1.0f, 2.0f, -1.0f },
        { 0.0f, -1.0f, 2.0f }
    };

    solver3.Solve(a);
    auto v = solver3.Eigenvectors();
    auto vtv = v.Transpose() * v;

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(vtv.at(i, j), (i == j) ? 1.0f : 0.0f, 1e-5f);
}

TEST_F(TestJacobiEigenSolver, satisfies_eigen_equation)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, -1.0f, 0.0f },
        { -1.0f, 2.0f, -1.0f },
        { 0.0f, -1.0f, 2.0f }
    };

    solver3.Solve(a);
    auto v = solver3.Eigenvectors();

    for (std::size_t k = 0; k < 3; ++k)
    {
        float lambda = solver3.Eigenvalues().at(k, 0);
        for (std::size_t i = 0; i < 3; ++i)
        {
            float av{ 0.0f };
            for (std::size_t j = 0; j < 3; ++j)
                av += a.at(i, j) * v.at(j, k);

            EXPECT_NEAR(av, lambda * v.at(i, k), 1e-5f);
        }
    }
}

TEST_F(TestJacobiEigenSolver, reconstructs_matrix_from_spectral_decomposition)
{
    math::Matrix<float, 3, 3> a{
        { 6.0f, 2.0f, 1.0f },
        { 2.0f, 3.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    };

    solver3.Solve(a);
    auto v = solver3.Eigenvectors();

    math::Matrix<float, 3, 3> d{};
    for (std::size_t i = 0; i < 3; ++i)
        d.at(i, i) = solver3.Eigenvalues().at(i, 0);

    auto reconstructed = v * d * v.Transpose();

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(reconstructed.at(i, j), a.at(i, j), 1e-4f);
}

TEST_F(TestJacobiEigenSolver, sweeps_is_zero_for_already_diagonal_input)
{
    math::Matrix<float, 3, 3> a{
        { 5.0f, 0.0f, 0.0f },
        { 0.0f, 2.0f, 0.0f },
        { 0.0f, 0.0f, 8.0f }
    };

    EXPECT_TRUE(solver3.Solve(a));
    EXPECT_EQ(solver3.Sweeps(), std::size_t{ 0 });
}

TEST_F(TestJacobiEigenSolver, sweeps_nonzero_and_bounded_after_off_diagonal_solve)
{
    math::Matrix<float, 2, 2> a{
        { 4.0f, 3.0f },
        { 3.0f, 4.0f }
    };

    EXPECT_TRUE(solver2.Solve(a));
    EXPECT_GT(solver2.Sweeps(), std::size_t{ 0 });
    EXPECT_LT(solver2.Sweeps(), std::size_t{ 50 });
}

TEST_F(TestJacobiEigenSolver, four_by_four_discrete_laplacian_eigenvalues)
{
    math::Matrix<float, 4, 4> a{
        { 2.0f, -1.0f, 0.0f, 0.0f },
        { -1.0f, 2.0f, -1.0f, 0.0f },
        { 0.0f, -1.0f, 2.0f, -1.0f },
        { 0.0f, 0.0f, -1.0f, 2.0f }
    };

    EXPECT_TRUE(solver4.Solve(a));

    EXPECT_NEAR(solver4.Eigenvalues().at(0, 0), 0.38197f, 1e-4f);
    EXPECT_NEAR(solver4.Eigenvalues().at(1, 0), 1.38197f, 1e-4f);
    EXPECT_NEAR(solver4.Eigenvalues().at(2, 0), 2.61803f, 1e-4f);
    EXPECT_NEAR(solver4.Eigenvalues().at(3, 0), 3.61803f, 1e-4f);
}

TEST_F(TestJacobiEigenSolver, determinism_same_input_produces_identical_output)
{
    math::Matrix<float, 3, 3> a{
        { 3.0f, 1.0f, 0.5f },
        { 1.0f, 5.0f, 2.0f },
        { 0.5f, 2.0f, 4.0f }
    };

    solver3.Solve(a);
    float ev0 = solver3.Eigenvalues().at(0, 0);
    float ev1 = solver3.Eigenvalues().at(1, 0);
    float ev2 = solver3.Eigenvalues().at(2, 0);

    solver3.Solve(a);

    EXPECT_FLOAT_EQ(solver3.Eigenvalues().at(0, 0), ev0);
    EXPECT_FLOAT_EQ(solver3.Eigenvalues().at(1, 0), ev1);
    EXPECT_FLOAT_EQ(solver3.Eigenvalues().at(2, 0), ev2);
}
