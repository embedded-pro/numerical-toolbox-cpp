#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestLuDecomposition : public ::testing::Test
    {
    protected:
        solvers::LuDecomposition<float, 3> lu{};
        solvers::LuDecomposition<float, 4> lu4{};
    };
}

TEST_F(TestLuDecomposition, reconstructs_PA_equals_LU)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);

    auto pa = lu.P() * a;
    auto reconstructed = lu.L() * lu.U();

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(pa.at(i, j), reconstructed.at(i, j), 1e-5f);
}

TEST_F(TestLuDecomposition, solves_linear_system)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    math::Vector<float, 3> b{ { 8.0f }, { -11.0f }, { -3.0f } };

    lu.Decompose(a);
    auto x = lu.Solve(b);

    EXPECT_NEAR(x.at(0, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(x.at(1, 0), 3.0f, 1e-5f);
    EXPECT_NEAR(x.at(2, 0), -1.0f, 1e-5f);
}

TEST_F(TestLuDecomposition, determinant_matches_known)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);

    EXPECT_NEAR(lu.Determinant(), -1.0f, 1e-4f);
}

TEST_F(TestLuDecomposition, inverse_times_A_is_identity)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);
    auto inv = lu.Inverse();
    auto product = a * inv;

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(product.at(i, j), (i == j) ? 1.0f : 0.0f, 1e-5f);
}

TEST_F(TestLuDecomposition, pivoting_handles_zero_leading_pivot)
{
    math::Matrix<float, 3, 3> a{
        { 0.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 9.0f }
    };

    bool ok = lu.Decompose(a);
    EXPECT_TRUE(ok);

    math::Vector<float, 3> b{ { 1.0f }, { 2.0f }, { 3.0f } };
    auto x = lu.Solve(b);
    auto residual = a * x;

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(residual.at(i, 0), b.at(i, 0), 1e-4f);
}

TEST_F(TestLuDecomposition, singular_matrix_returns_false)
{
    math::Matrix<float, 3, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 9.0f }
    };

    bool ok = lu.Decompose(a);

    EXPECT_FALSE(ok);
}

TEST_F(TestLuDecomposition, multiple_rhs_reuse_factorization)
{
    math::Matrix<float, 4, 4> a{
        { 2.0f, 1.0f, 0.0f, 0.0f },
        { 1.0f, 3.0f, 1.0f, 0.0f },
        { 0.0f, 1.0f, 4.0f, 1.0f },
        { 0.0f, 0.0f, 1.0f, 2.0f }
    };

    math::Vector<float, 4> b1{ { 1.0f }, { 2.0f }, { 3.0f }, { 4.0f } };
    math::Vector<float, 4> b2{ { 5.0f }, { 6.0f }, { 7.0f }, { 8.0f } };

    lu4.Decompose(a);
    auto x1 = lu4.Solve(b1);
    auto x2 = lu4.Solve(b2);

    EXPECT_NEAR(x1.at(0, 0), 0.2258065f, 1e-4f);
    EXPECT_NEAR(x1.at(1, 0), 0.5483871f, 1e-4f);
    EXPECT_NEAR(x1.at(2, 0), 0.1290323f, 1e-4f);
    EXPECT_NEAR(x1.at(3, 0), 1.9354839f, 1e-4f);

    EXPECT_NEAR(x2.at(0, 0), 1.9032258f, 1e-4f);
    EXPECT_NEAR(x2.at(1, 0), 1.1935484f, 1e-4f);
    EXPECT_NEAR(x2.at(2, 0), 0.5161290f, 1e-4f);
    EXPECT_NEAR(x2.at(3, 0), 3.7419355f, 1e-4f);
}

TEST_F(TestLuDecomposition, matches_gaussian_elimination)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    math::Vector<float, 3> b{ { 8.0f }, { -11.0f }, { -3.0f } };

    lu.Decompose(a);
    auto xLu = lu.Solve(b);

    solvers::GaussianElimination<float, 3> ge{};
    auto xGe = ge.Solve(a, b);

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(xLu.at(i, 0), xGe.at(i, 0), math::Tolerance<float>());
}

TEST_F(TestLuDecomposition, is_singular_false_after_successful_decompose)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);

    EXPECT_FALSE(lu.IsSingular());
}

TEST_F(TestLuDecomposition, is_singular_true_after_singular_decompose)
{
    math::Matrix<float, 3, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 9.0f }
    };

    lu.Decompose(a);

    EXPECT_TRUE(lu.IsSingular());
}

TEST_F(TestLuDecomposition, P_is_valid_permutation_matrix)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);
    auto p = lu.P();

    for (std::size_t i = 0; i < 3; ++i)
    {
        float rowSum{ 0.0f };
        float colSum{ 0.0f };
        for (std::size_t j = 0; j < 3; ++j)
        {
            rowSum += p.at(i, j);
            colSum += p.at(j, i);
        }
        EXPECT_NEAR(rowSum, 1.0f, math::Tolerance<float>());
        EXPECT_NEAR(colSum, 1.0f, math::Tolerance<float>());
    }
}

TEST_F(TestLuDecomposition, L_has_unit_diagonal_and_lower_triangular_structure)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);
    auto l = lu.L();

    for (std::size_t i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(l.at(i, i), 1.0f, math::Tolerance<float>());
        for (std::size_t j = i + 1; j < 3; ++j)
            EXPECT_NEAR(l.at(i, j), 0.0f, math::Tolerance<float>());
    }
}

TEST_F(TestLuDecomposition, U_has_upper_triangular_structure)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    lu.Decompose(a);
    auto u = lu.U();

    for (std::size_t i = 1; i < 3; ++i)
        for (std::size_t j = 0; j < i; ++j)
            EXPECT_NEAR(u.at(i, j), 0.0f, math::Tolerance<float>());
}

TEST_F(TestLuDecomposition, determinant_identity_matrix_is_one)
{
    math::Matrix<float, 3, 3> identity{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    lu.Decompose(identity);

    EXPECT_NEAR(lu.Determinant(), 1.0f, math::Tolerance<float>());
}

TEST_F(TestLuDecomposition, determinant_diagonal_matrix_is_product_of_diagonal)
{
    math::Matrix<float, 3, 3> d{
        { 2.0f, 0.0f, 0.0f },
        { 0.0f, 3.0f, 0.0f },
        { 0.0f, 0.0f, 5.0f }
    };

    lu.Decompose(d);

    EXPECT_NEAR(lu.Determinant(), 30.0f, math::Tolerance<float>());
}

TEST_F(TestLuDecomposition, hilbert_3x3_residual_bounded_by_conditioning)
{
    math::Matrix<float, 3, 3> h{
        { 1.0f, 0.5f, 1.0f / 3.0f },
        { 0.5f, 1.0f / 3.0f, 0.25f },
        { 1.0f / 3.0f, 0.25f, 0.2f }
    };

    math::Vector<float, 3> b{ { 1.0f }, { 1.0f }, { 1.0f } };

    lu.Decompose(h);
    auto x = lu.Solve(b);
    auto residual = h * x;

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(residual.at(i, 0), b.at(i, 0), 1e-2f);
}

TEST_F(TestLuDecomposition, decompose_is_deterministic)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, -1.0f },
        { -3.0f, -1.0f, 2.0f },
        { -2.0f, 1.0f, 2.0f }
    };

    solvers::LuDecomposition<float, 3> lu2{};
    lu.Decompose(a);
    lu2.Decompose(a);

    auto l1 = lu.L();
    auto l2 = lu2.L();
    auto u1 = lu.U();
    auto u2 = lu2.U();

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            EXPECT_FLOAT_EQ(l1.at(i, j), l2.at(i, j));
            EXPECT_FLOAT_EQ(u1.at(i, j), u2.at(i, j));
        }
}
