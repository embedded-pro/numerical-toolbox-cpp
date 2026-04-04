#include "numerical/math/QNumber.hpp"
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
}

TYPED_TEST(TestGaussianElimination, solve_returns_vector_of_correct_size)
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

namespace
{
    class TestGaussianEliminationFloat
        : public ::testing::Test
    {
    };
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
