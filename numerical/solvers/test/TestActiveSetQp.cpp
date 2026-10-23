#include "numerical/solvers/ActiveSetQp.hpp"
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using WithEquality = solvers::ActiveSetQp<float, 2, 1, 2>;
    using BoxOnly = solvers::ActiveSetQp<float, 2, 0, 4>;
    using Vector2 = math::Vector<float, 2>;
    using Vector4 = math::Vector<float, 4>;
    using Rows4 = math::Matrix<float, 4, 2>;

    class TestActiveSetQp : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        WithEquality withEquality;
        BoxOnly boxOnly;

        static BoxOnly::Program LeastDistance(const Vector2& reference, const Rows4& rows, const Vector4& bounds)
        {
            return { math::SquareMatrix<float, 2>::Identity(), reference * -1.0f, rows, bounds };
        }

        static void ExpectNear(const Vector2& actual, float x0, float x1)
        {
            EXPECT_NEAR(actual[0], x0, tolerance);
            EXPECT_NEAR(actual[1], x1, tolerance);
        }
    };
}

TEST_F(TestActiveSetQp, inactive_constraints_return_the_unconstrained_minimum)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{ 1.0f, 1.0f }, Rows4{ { 1.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, -1.0f } }, Vector4{ -5.0f, -5.0f, -5.0f, -5.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 1.0f, 1.0f);
    for (std::size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(result.multipliers[i], 0.0f);
        EXPECT_FALSE(result.active[i]);
    }
}

TEST_F(TestActiveSetQp, equality_constraint_is_met_with_its_multiplier)
{
    const WithEquality::Program qp{ math::SquareMatrix<float, 2>::Identity(), Vector2{}, math::Matrix<float, 1, 2>{ 1.0f, 1.0f }, math::Vector<float, 1>{ 1.0f },
        math::Matrix<float, 2, 2>{ { 1.0f, 0.0f }, { 0.0f, 1.0f } }, Vector2{ -5.0f, -5.0f } };

    const auto result{ withEquality.Solve(qp) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 0.5f, 0.5f);
    EXPECT_NEAR(result.multipliers[0], 0.5f, tolerance);
    EXPECT_TRUE(result.active[0]);
}

TEST_F(TestActiveSetQp, violated_inequality_becomes_active)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{ 1.0f, 1.0f }, Rows4{ { -1.0f, -1.0f }, { 1.0f, 0.0f }, { 0.0f, 1.0f }, { -1.0f, 0.0f } }, Vector4{ -1.0f, -5.0f, -5.0f, -5.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 0.5f, 0.5f);
    EXPECT_NEAR(result.multipliers[0], 0.5f, tolerance);
    EXPECT_TRUE(result.active[0]);
}

TEST_F(TestActiveSetQp, box_bounds_clip_to_the_nearest_corner)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{ 2.0f, -1.0f }, Rows4{ { 1.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, -1.0f } }, Vector4{ 0.0f, -1.0f, 0.0f, -1.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 1.0f, 0.0f);
    EXPECT_NEAR(result.multipliers[0], 0.0f, tolerance);
    EXPECT_NEAR(result.multipliers[1], 1.0f, tolerance);
    EXPECT_NEAR(result.multipliers[2], 1.0f, tolerance);
    EXPECT_NEAR(result.multipliers[3], 0.0f, tolerance);
}

TEST_F(TestActiveSetQp, blocking_constraint_is_dropped_on_the_way)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{}, Rows4{ { 0.0f, 1.0f }, { 0.1f, 0.5f }, { 1.0f, 0.0f }, { 0.0f, 1.0f } }, Vector4{ 1.0f, 0.9f, -5.0f, -5.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 0.346154f, 1.730769f);
    EXPECT_FALSE(result.active[0]);
    EXPECT_TRUE(result.active[1]);
    EXPECT_NEAR(result.multipliers[0], 0.0f, tolerance);
    EXPECT_NEAR(result.multipliers[1], 3.461538f, 1e-4f);
}

TEST_F(TestActiveSetQp, contradictory_constraints_are_infeasible)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{}, Rows4{ { 1.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, -1.0f } }, Vector4{ 1.0f, 0.0f, -5.0f, -5.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Infeasible);
}

TEST_F(TestActiveSetQp, indefinite_hessian_is_rejected)
{
    auto qp{ LeastDistance(Vector2{}, Rows4{ { 1.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, -1.0f } }, Vector4{ -5.0f, -5.0f, -5.0f, -5.0f }) };
    qp.hessian.at(1, 1) = -1.0f;

    EXPECT_EQ(boxOnly.Solve(qp).status, solvers::QpStatus::NotPositiveDefinite);
}

TEST_F(TestActiveSetQp, redundant_constraints_do_not_break_the_solve)
{
    const auto result{ boxOnly.Solve(LeastDistance(Vector2{ 1.0f, 1.0f }, Rows4{ { -1.0f, -1.0f }, { -1.0f, -1.0f }, { 1.0f, 0.0f }, { 0.0f, 1.0f } }, Vector4{ -1.0f, -1.0f, -5.0f, -5.0f })) };

    EXPECT_EQ(result.status, solvers::QpStatus::Optimal);
    ExpectNear(result.x, 0.5f, 0.5f);
    EXPECT_NEAR(result.multipliers[0] + result.multipliers[1], 0.5f, tolerance);
}
