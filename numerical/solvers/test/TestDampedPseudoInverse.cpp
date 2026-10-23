#include "numerical/solvers/DampedPseudoInverse.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Matrix23 = math::Matrix<float, 2, 3>;
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;

    class TestDampedPseudoInverse : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        const Matrix23 a{ { 1.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f } };
        const Vector2 b{ 1.0f, 2.0f };
        solvers::DampedPseudoInverse<float, 2, 3> exact{ 0.0f };
        solvers::DampedPseudoInverse<float, 2, 3> damped{ 0.1f };

        void SetUp() override
        {
            ASSERT_TRUE(exact.Factor(a));
            ASSERT_TRUE(damped.Factor(a));
        }

        template<std::size_t Rows, std::size_t Cols>
        static void ExpectNear(const math::Matrix<float, Rows, Cols>& actual, const math::Matrix<float, Rows, Cols>& expected, float eps = tolerance)
        {
            for (std::size_t r = 0; r < Rows; ++r)
                for (std::size_t c = 0; c < Cols; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), eps) << "(" << r << ", " << c << ")";
        }

        static float Norm(const Vector2& v)
        {
            return std::sqrt(v[0] * v[0] + v[1] * v[1]);
        }
    };
}

TEST_F(TestDampedPseudoInverse, undamped_solve_is_minimum_norm_exact_solution)
{
    const Vector3 x{ exact.Solve(b) };

    ExpectNear(x, Vector3{ 0.0f, 1.0f, 1.0f });
    ExpectNear(Vector2{ a * x }, b);
}

TEST_F(TestDampedPseudoInverse, damped_solve_error_is_bounded)
{
    const Vector3 x{ damped.Solve(b) };
    const float residual{ Norm(a * x - b) };

    ExpectNear(x, Vector3{ 0.003289f, 0.993388f, 0.996678f });
    EXPECT_NEAR(residual, 0.009934f, tolerance);
    EXPECT_LE(residual, 0.1f * 0.1f / 1.0f * Norm(b));
}

TEST_F(TestDampedPseudoInverse, projection_is_invisible_to_the_matrix_despite_damping)
{
    const Vector3 projected{ damped.Project(Vector3{ 1.0f, 0.0f, 0.0f }) };

    ExpectNear(projected, Vector3{ 1.0f / 3.0f, 1.0f / 3.0f, -1.0f / 3.0f });
    ExpectNear(Vector2{ a * projected }, Vector2{});
}

TEST_F(TestDampedPseudoInverse, projector_is_an_orthogonal_projector)
{
    const auto p{ damped.NullSpaceProjector() };

    ExpectNear(p * p, p);
    ExpectNear(p.Transpose(), p);
    EXPECT_NEAR(p.Trace(), 1.0f, tolerance);
}

TEST_F(TestDampedPseudoInverse, undamped_inverse_is_a_right_inverse)
{
    ExpectNear(a * exact.Inverse(), math::SquareMatrix<float, 2>::Identity());
}

TEST_F(TestDampedPseudoInverse, solve_has_no_null_space_component)
{
    ExpectNear(damped.Project(damped.Solve(b)), Vector3{});
}

TEST_F(TestDampedPseudoInverse, rank_loss_grows_the_null_space)
{
    ASSERT_TRUE(exact.Factor(Matrix23{ { 1.0f, 1.0f, 0.0f }, { 2.0f, 2.0f, 0.0f } }));

    EXPECT_EQ(exact.Rank(), 1);
    EXPECT_NEAR(exact.NullSpaceProjector().Trace(), 2.0f, tolerance);
    ExpectNear(exact.Solve(Vector2{ 1.0f, 2.0f }), Vector3{ 0.5f, 0.5f, 0.0f });
}
