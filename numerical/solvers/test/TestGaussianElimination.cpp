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
        static T MakeValue(float f)
        {
            f = std::max(std::min(f, 0.9999f), -0.9999f);
            if constexpr (std::is_same_v<T, float>)
                return f;
            else
                return T(f);
        }

        template<std::size_t N>
        static bool AreVectorsNear(const math::Vector<T, N>& a, const math::Vector<T, N>& b, float epsilon = 1e-2f)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                float va, vb;
                if constexpr (std::is_same_v<T, float>)
                {
                    va = a.at(i, 0);
                    vb = b.at(i, 0);
                }
                else
                {
                    va = a.at(i, 0).ToFloat();
                    vb = b.at(i, 0).ToFloat();
                }

                if (std::abs(va - vb) > epsilon)
                    return false;
            }
            return true;
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestGaussianElimination, TestTypes);
}

TYPED_TEST(TestGaussianElimination, identity_system_returns_rhs)
{
    math::SquareMatrix<TypeParam, 2> A{
        { this->MakeValue(0.9999f), this->MakeValue(0.0f) },
        { this->MakeValue(0.0f), this->MakeValue(0.9999f) }
    };

    math::Vector<TypeParam, 2> b{
        { this->MakeValue(0.3f) },
        { this->MakeValue(0.5f) }
    };

    solvers::GaussianElimination<TypeParam, 2> solver;
    auto x = solver.Solve(A, b);

    EXPECT_TRUE(this->template AreVectorsNear<2>(x, b, 1e-2f));
}

TYPED_TEST(TestGaussianElimination, solves_2x2_system)
{
    math::SquareMatrix<TypeParam, 2> A{
        { this->MakeValue(0.5f), this->MakeValue(0.2f) },
        { this->MakeValue(0.1f), this->MakeValue(0.4f) }
    };

    math::Vector<TypeParam, 2> b{
        { this->MakeValue(0.29f) },
        { this->MakeValue(0.22f) }
    };

    solvers::GaussianElimination<TypeParam, 2> solver;
    auto x = solver.Solve(A, b);

    math::Vector<TypeParam, 2> expected{
        { this->MakeValue(0.4f) },
        { this->MakeValue(0.45f) }
    };

    EXPECT_TRUE(this->template AreVectorsNear<2>(x, expected, 0.02f));
}

TYPED_TEST(TestGaussianElimination, solves_3x3_system)
{
    math::SquareMatrix<TypeParam, 3> A{
        { this->MakeValue(0.5f), this->MakeValue(0.1f), this->MakeValue(0.1f) },
        { this->MakeValue(0.1f), this->MakeValue(0.4f), this->MakeValue(0.1f) },
        { this->MakeValue(0.1f), this->MakeValue(0.1f), this->MakeValue(0.3f) }
    };

    math::Vector<TypeParam, 3> b{
        { this->MakeValue(0.13f) },
        { this->MakeValue(0.11f) },
        { this->MakeValue(0.07f) }
    };

    solvers::GaussianElimination<TypeParam, 3> solver;
    auto x = solver.Solve(A, b);

    math::Vector<TypeParam, 3> expected{
        { this->MakeValue(0.2f) },
        { this->MakeValue(0.2f) },
        { this->MakeValue(0.1f) }
    };

    EXPECT_TRUE(this->template AreVectorsNear<3>(x, expected, 0.05f));
}

TYPED_TEST(TestGaussianElimination, pivot_selects_largest_element)
{
    math::SquareMatrix<TypeParam, 2> A{
        { this->MakeValue(0.01f), this->MakeValue(0.5f) },
        { this->MakeValue(0.5f), this->MakeValue(0.1f) }
    };

    math::Vector<TypeParam, 2> b{
        { this->MakeValue(0.255f) },
        { this->MakeValue(0.3f) }
    };

    solvers::GaussianElimination<TypeParam, 2> solver;
    auto x = solver.Solve(A, b);

    math::Vector<TypeParam, 2> expected{
        { this->MakeValue(0.5f) },
        { this->MakeValue(0.5f) }
    };

    EXPECT_TRUE(this->template AreVectorsNear<2>(x, expected, 0.05f));
}

TEST(TestGaussianEliminationFloat, solve_system_multiple_rhs)
{
    math::SquareMatrix<float, 2> A{
        { 0.5f, 0.2f },
        { 0.1f, 0.4f }
    };

    math::Matrix<float, 2, 2> B{
        { 0.29f, 0.5f },
        { 0.22f, 0.1f }
    };

    auto X = solvers::SolveSystem<float, 2, 2>(A, B);

    EXPECT_NEAR(X.at(0, 0), 0.4f, 0.01f);
    EXPECT_NEAR(X.at(1, 0), 0.45f, 0.01f);
}
