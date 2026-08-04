#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestGaussianElimination
        : public ::testing::Test
    {
    protected:
        solvers::GaussianElimination<T, 3> solver;
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestGaussianElimination, TestTypes);

    class TestGaussianEliminationFloat
        : public ::testing::Test
    {
    protected:
        solvers::GaussianElimination<float, 2> solver;
    };

    class TestGaussianEliminationFloat3
        : public ::testing::Test
    {
    protected:
        solvers::GaussianElimination<float, 3> solver3;
    };
}

TYPED_TEST(TestGaussianElimination, solve_identity_matrix_returns_rhs)
{
    auto a = math::SquareMatrix<TypeParam, 3>::Identity();
    math::Vector<TypeParam, 3> b{ { TypeParam(0.1f) }, { TypeParam(0.2f) }, { TypeParam(0.3f) } };

    auto result = this->solver.Solve(a, b);

    EXPECT_NEAR(math::ToFloat(result.at(0, 0)), 0.1f, 0.01f);
    EXPECT_NEAR(math::ToFloat(result.at(1, 0)), 0.2f, 0.01f);
    EXPECT_NEAR(math::ToFloat(result.at(2, 0)), 0.3f, 0.01f);
}

TYPED_TEST(TestGaussianElimination, solve_applies_pivot_when_diagonal_is_small)
{
    math::SquareMatrix<TypeParam, 3> a{
        { TypeParam(0.01f), TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.5f), TypeParam(0.1f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.0f), TypeParam(0.5f) }
    };
    math::Vector<TypeParam, 3> b{ { TypeParam(0.25f) }, { TypeParam(0.3f) }, { TypeParam(0.1f) } };

    auto result = this->solver.Solve(a, b);

    EXPECT_NEAR(math::ToFloat(result.at(2, 0)), 0.2f, 0.02f);
}

TYPED_TEST(TestGaussianElimination, solve_eliminates_below_diagonal)
{
    math::SquareMatrix<TypeParam, 3> a{
        { TypeParam(0.5f), TypeParam(0.1f), TypeParam(0.0f) },
        { TypeParam(0.25f), TypeParam(0.4f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.0f), TypeParam(0.5f) }
    };
    math::Vector<TypeParam, 3> b{ { TypeParam(0.07f) }, { TypeParam(0.115f) }, { TypeParam(0.15f) } };

    auto result = this->solver.Solve(a, b);

    EXPECT_NEAR(math::ToFloat(result.at(0, 0)), 0.1f, 0.05f);
    EXPECT_NEAR(math::ToFloat(result.at(1, 0)), 0.2f, 0.05f);
    EXPECT_NEAR(math::ToFloat(result.at(2, 0)), 0.3f, 0.05f);
}

TYPED_TEST(TestGaussianElimination, solve_back_substitutes_upper_triangular)
{
    math::SquareMatrix<TypeParam, 3> a{
        { TypeParam(0.5f), TypeParam(0.1f), TypeParam(0.2f) },
        { TypeParam(0.0f), TypeParam(0.4f), TypeParam(0.1f) },
        { TypeParam(0.0f), TypeParam(0.0f), TypeParam(0.3f) }
    };
    math::Vector<TypeParam, 3> b{ { TypeParam(0.13f) }, { TypeParam(0.11f) }, { TypeParam(0.09f) } };

    auto result = this->solver.Solve(a, b);

    EXPECT_NEAR(math::ToFloat(result.at(2, 0)), 0.3f, 0.02f);
    EXPECT_NEAR(math::ToFloat(result.at(1, 0)), 0.2f, 0.02f);
    EXPECT_NEAR(math::ToFloat(result.at(0, 0)), 0.1f, 0.02f);
}

TEST_F(TestGaussianEliminationFloat, solve_system_delegates_per_column)
{
    math::SquareMatrix<float, 2> a{
        { 0.5f, 0.2f },
        { 0.1f, 0.4f }
    };
    math::Matrix<float, 2, 2> b{
        { 0.29f, 0.5f },
        { 0.22f, 0.1f }
    };

    auto result = solvers::SolveSystem<float, 2, 2>(a, b);

    EXPECT_NEAR(result.at(0, 0), 0.4f, 0.01f);
    EXPECT_NEAR(result.at(1, 0), 0.45f, 0.01f);
    EXPECT_NEAR(result.at(0, 1), 1.0f, 0.01f);
    EXPECT_NEAR(result.at(1, 1), 0.0f, 0.01f);
}

TEST_F(TestGaussianEliminationFloat3, general_3x3_well_conditioned_full_solve)
{
    math::SquareMatrix<float, 3> a{
        { 2.0f,  1.0f, -1.0f },
        { -3.0f, -1.0f,  2.0f },
        { -2.0f,  1.0f,  2.0f }
    };
    math::Vector<float, 3> b{ { 8.0f }, { -11.0f }, { -3.0f } };

    auto x = solver3.Solve(a, b);

    EXPECT_NEAR(x.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(2, 0), -1.0f, math::Tolerance<float>());
}

TEST_F(TestGaussianEliminationFloat3, solve_system_with_identity_rhs_yields_inverse)
{
    math::SquareMatrix<float, 3> a{
        { 2.0f, 1.0f, 0.0f },
        { 1.0f, 3.0f, 1.0f },
        { 0.0f, 1.0f, 2.0f }
    };
    auto identity = math::SquareMatrix<float, 3>::Identity();

    auto inv = solvers::SolveSystem<float, 3, 3>(a, identity);
    auto product = a * inv;

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(product.at(i, j), (i == j) ? 1.0f : 0.0f, math::Tolerance<float>());
}

TEST_F(TestGaussianEliminationFloat, make_gaussian_elimination_factory_produces_working_solver)
{
    auto factorySolver = solvers::MakeGaussianElimination<float, 2>();

    math::SquareMatrix<float, 2> a{
        { 3.0f, 1.0f },
        { 1.0f, 2.0f }
    };
    math::Vector<float, 2> b{ { 9.0f }, { 8.0f } };

    auto x = factorySolver.Solve(a, b);

    EXPECT_NEAR(x.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 3.0f, math::Tolerance<float>());
}
