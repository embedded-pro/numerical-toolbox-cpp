#include "numerical/optimization/LevenbergMarquardt.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    using Vector1 = math::Vector<float, 1>;
    using Vector2 = math::Vector<float, 2>;
    using Matrix21 = math::Matrix<float, 2, 1>;
    using Matrix22 = math::Matrix<float, 2, 2>;

    MATCHER_P(MatrixNear, expected, "")
    {
        for (std::size_t r = 0; r < 2; ++r)
            for (std::size_t c = 0; c < 2; ++c)
                if (std::abs(arg.at(r, c) - expected.at(r, c)) > 1e-5f)
                    return false;

        return true;
    }

    class RosenbrockResiduals
        : public optimization::LeastSquaresProblem<float, Vector2, 2, 2>
    {
    public:
        Vector2 Residual(const Vector2& x) const override
        {
            return Vector2{ 10.0f * (x[1] - x[0] * x[0]), 1.0f - x[0] };
        }

        Matrix22 Jacobian(const Vector2& x) const override
        {
            return Matrix22{ { -20.0f * x[0], 10.0f }, { -1.0f, 0.0f } };
        }

        Vector2 Retract(const Vector2& x, const Vector2& delta) const override
        {
            return x + delta;
        }
    };

    class UnitCircleAngle
        : public optimization::LeastSquaresProblem<float, float, 2, 1>
    {
    public:
        explicit UnitCircleAngle(const Vector2& target)
            : target{ target }
        {}

        Vector2 Residual(const float& theta) const override
        {
            return Vector2{ std::cos(theta) - target[0], std::sin(theta) - target[1] };
        }

        Matrix21 Jacobian(const float& theta) const override
        {
            return Matrix21{ -std::sin(theta), std::cos(theta) };
        }

        float Retract(const float& theta, const Vector1& delta) const override
        {
            const float wrapped{ std::remainder(theta + delta[0], 2.0f * std::numbers::pi_v<float>) };
            const float retracted{ wrapped <= -std::numbers::pi_v<float> ? wrapped + 2.0f * std::numbers::pi_v<float> : wrapped };
            if (retractions == 0)
                firstRetraction = retracted;
            largestMagnitude = std::max(largestMagnitude, std::abs(retracted));
            ++retractions;
            return retracted;
        }

        mutable float firstRetraction{ 0.0f };
        mutable float largestMagnitude{ 0.0f };
        mutable int retractions{ 0 };

    private:
        Vector2 target;
    };

    class ConstantResidual
        : public optimization::LeastSquaresProblem<float, Vector2, 2, 2>
    {
    public:
        Vector2 Residual(const Vector2&) const override
        {
            return Vector2{ 1.0f, 1.0f };
        }

        Matrix22 Jacobian(const Vector2&) const override
        {
            return Matrix22{};
        }

        Vector2 Retract(const Vector2& x, const Vector2& delta) const override
        {
            return x + delta;
        }
    };

    class MockDampingPolicy
        : public optimization::DampingPolicy<float, 2, 2>
    {
    public:
        MOCK_METHOD(float, DampingSquared, (const Matrix22& jacobian), (const, override));
    };

    class TestLevenbergMarquardt
        : public ::testing::Test
    {
    protected:
        RosenbrockResiduals rosenbrock;
        optimization::ConstantDamping<float, 2, 2> gaussNewton{ 0.0f };
        optimization::ConstantDamping<float, 2, 1> circleGaussNewton{ 0.0f };
        testing::StrictMock<MockDampingPolicy> policy;
        optimization::LevenbergMarquardtConfig<float> config{ 1e-4f, std::numeric_limits<float>::infinity(), 20 };
        const Vector2 start{ -1.2f, 1.0f };
    };
}

TEST_F(TestLevenbergMarquardt, gauss_newton_solves_rosenbrock_residuals_in_two_steps)
{
    const auto result{ optimization::LevenbergMarquardt<float, Vector2, 2, 2>{ rosenbrock, gaussNewton, config }.Solve(start) };

    EXPECT_NEAR(result.x[0], 1.0f, 1e-5f);
    EXPECT_NEAR(result.x[1], 1.0f, 1e-5f);
    EXPECT_TRUE(result.converged);
    EXPECT_EQ(result.iterations, 2);
}

TEST_F(TestLevenbergMarquardt, damping_policy_is_queried_with_the_current_jacobian)
{
    config.maxIterations = 1;
    EXPECT_CALL(policy, DampingSquared(MatrixNear(Matrix22{ { 24.0f, 10.0f }, { -1.0f, 0.0f } }))).WillOnce(testing::Return(0.0f));

    const auto result{ optimization::LevenbergMarquardt<float, Vector2, 2, 2>{ rosenbrock, policy, config }.Solve(start) };

    EXPECT_NEAR(result.x[0], 1.0f, 1e-5f);
    EXPECT_NEAR(result.x[1], -3.84f, 1e-4f);
    EXPECT_FALSE(result.converged);
}

TEST_F(TestLevenbergMarquardt, step_is_clamped_to_max_step)
{
    config.maxIterations = 1;
    config.maxStep = 1.0f;

    const auto result{ optimization::LevenbergMarquardt<float, Vector2, 2, 2>{ rosenbrock, gaussNewton, config }.Solve(start) };

    EXPECT_NEAR(result.x[0], -0.786197f, 1e-5f);
    EXPECT_NEAR(result.x[1], 0.089634f, 1e-5f);
}

TEST_F(TestLevenbergMarquardt, retraction_keeps_a_wrapped_state)
{
    const UnitCircleAngle circle{ Vector2{ std::cos(2.5f), std::sin(2.5f) } };

    const auto result{ optimization::LevenbergMarquardt<float, float, 2, 1>{ circle, circleGaussNewton, config }.Solve(-2.5f) };

    EXPECT_NEAR(result.x, 2.5f, 1e-4f);
    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(circle.firstRetraction, 2.824261f, 1e-5f);
    EXPECT_LE(circle.largestMagnitude, std::numbers::pi_v<float>);
}

TEST_F(TestLevenbergMarquardt, unreachable_target_reports_non_convergence)
{
    const UnitCircleAngle circle{ Vector2{ 2.0f, 0.0f } };
    const optimization::ConstantDamping<float, 2, 1> damped{ 1.0f };

    const auto result{ optimization::LevenbergMarquardt<float, float, 2, 1>{ circle, damped, config }.Solve(0.3f) };

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, config.maxIterations);
    EXPECT_NEAR(result.x, 0.0f, 1e-5f);
    EXPECT_NEAR(result.residualNorm, 1.0f, 1e-5f);
}

TEST_F(TestLevenbergMarquardt, singular_undamped_system_stops_without_dividing_by_zero)
{
    const ConstantResidual constant;

    const auto result{ optimization::LevenbergMarquardt<float, Vector2, 2, 2>{ constant, gaussNewton, config }.Solve(start) };

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_EQ(result.x[0], start[0]);
    EXPECT_EQ(result.x[1], start[1]);
}
