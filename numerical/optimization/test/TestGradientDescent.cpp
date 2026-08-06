#include "numerical/math/Tolerance.hpp"
#include "numerical/optimization/GradientDescent.hpp"
#include "numerical/optimization/ObjectiveFunction.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using Vector2f = math::Vector<float, 2>;

    class MockObjectiveFunction
        : public optimization::ObjectiveFunction<float, 2>
    {
    public:
        MOCK_METHOD(float, Cost, (const Vector2f& parameters), (override));
        MOCK_METHOD(Vector2f, Gradient, (const Vector2f& parameters), (override));
    };

    class QuadraticObjective
        : public optimization::ObjectiveFunction<float, 2>
    {
    public:
        float Cost(const Vector2f& p) override
        {
            return p[0] * p[0] + p[1] * p[1];
        }

        Vector2f Gradient(const Vector2f& p) override
        {
            Vector2f g;
            g[0] = 2.0f * p[0];
            g[1] = 2.0f * p[1];
            return g;
        }
    };

    class RosenbrockObjective
        : public optimization::ObjectiveFunction<float, 2>
    {
    public:
        float Cost(const Vector2f& p) override
        {
            const float a = 1.0f - p[0];
            const float b = p[1] - p[0] * p[0];
            return a * a + 100.0f * b * b;
        }

        Vector2f Gradient(const Vector2f& p) override
        {
            Vector2f g;
            g[0] = -2.0f * (1.0f - p[0]) - 400.0f * p[0] * (p[1] - p[0] * p[0]);
            g[1] = 200.0f * (p[1] - p[0] * p[0]);
            return g;
        }
    };

    class ZeroGradientObjective
        : public optimization::ObjectiveFunction<float, 2>
    {
    public:
        float Cost(const Vector2f&) override { return 42.0f; }

        Vector2f Gradient(const Vector2f&) override
        {
            Vector2f g;
            g[0] = 0.0f;
            g[1] = 0.0f;
            return g;
        }
    };

    Vector2f MakeVector(float x, float y)
    {
        Vector2f v;
        v[0] = x;
        v[1] = y;
        return v;
    }

    optimization::GradientDescent<float, 2>::Parameters MakeParams(float lr, std::size_t iters)
    {
        return { lr, iters };
    }

    class TestGradientDescent
        : public ::testing::Test
    {
    protected:
        testing::StrictMock<MockObjectiveFunction> objective;
        QuadraticObjective quadratic;
        RosenbrockObjective rosenbrock;
        ZeroGradientObjective zeroGrad;
    };
}

TEST_F(TestGradientDescent, performs_expected_number_of_iterations)
{
    constexpr std::size_t N = 5;
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.1f, N) };

    EXPECT_CALL(objective, Cost(testing::_))
        .Times(N + 1)
        .WillRepeatedly(testing::Return(0.5f));
    EXPECT_CALL(objective, Gradient(testing::_))
        .Times(N)
        .WillRepeatedly(testing::Return(MakeVector(0.01f, 0.01f)));

    const auto& result = optimizer.Minimize(MakeVector(0.0f, 0.0f), objective);

    EXPECT_EQ(result.iterations, N);
}

TEST_F(TestGradientDescent, parameters_updated_by_gradient_step)
{
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.1f, 1) };

    EXPECT_CALL(objective, Cost(testing::_))
        .Times(2)
        .WillRepeatedly(testing::Return(0.0f));
    EXPECT_CALL(objective, Gradient(testing::_))
        .WillOnce(testing::Return(MakeVector(0.1f, 0.2f)));

    const auto& result = optimizer.Minimize(MakeVector(0.5f, 0.4f), objective);

    EXPECT_NEAR(result.parameters[0], 0.49f, 1e-5f);
    EXPECT_NEAR(result.parameters[1], 0.38f, 1e-5f);
}

TEST_F(TestGradientDescent, result_stores_final_cost)
{
    constexpr float kFinalCost = 0.123f;
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.1f, 3) };

    {
        testing::InSequence seq;
        EXPECT_CALL(objective, Cost(testing::_)).WillOnce(testing::Return(1.0f));
        EXPECT_CALL(objective, Gradient(testing::_)).WillOnce(testing::Return(MakeVector(0.01f, 0.01f)));
        EXPECT_CALL(objective, Cost(testing::_)).WillOnce(testing::Return(0.5f));
        EXPECT_CALL(objective, Gradient(testing::_)).WillOnce(testing::Return(MakeVector(0.01f, 0.01f)));
        EXPECT_CALL(objective, Cost(testing::_)).WillOnce(testing::Return(0.25f));
        EXPECT_CALL(objective, Gradient(testing::_)).WillOnce(testing::Return(MakeVector(0.01f, 0.01f)));
        EXPECT_CALL(objective, Cost(testing::_)).WillOnce(testing::Return(kFinalCost));
    }

    const auto& result = optimizer.Minimize(MakeVector(0.0f, 0.0f), objective);

    EXPECT_NEAR(result.finalCost, kFinalCost, math::Tolerance<float>());
}

TEST_F(TestGradientDescent, converges_to_quadratic_minimum)
{
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.5f, 1) };

    const auto& result = optimizer.Minimize(MakeVector(3.0f, 4.0f), quadratic);

    EXPECT_NEAR(result.parameters[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.parameters[1], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.finalCost, 0.0f, math::Tolerance<float>());
}

TEST_F(TestGradientDescent, cost_decreases_monotonically_on_convex_function)
{
    constexpr std::size_t Steps = 20;
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.1f, 1) };

    float prevCost = quadratic.Cost(MakeVector(5.0f, 5.0f));
    auto params = MakeVector(5.0f, 5.0f);

    for (std::size_t i = 0; i < Steps; ++i)
    {
        optimization::GradientDescent<float, 2> stepOptimizer{ MakeParams(0.1f, 1) };
        const auto& result = stepOptimizer.Minimize(params, quadratic);
        EXPECT_LT(result.finalCost, prevCost);
        prevCost = result.finalCost;
        params = result.parameters;
    }
}

TEST_F(TestGradientDescent, analytic_gradient_matches_finite_difference)
{
    constexpr float h = 1e-3f;
    const auto p = MakeVector(1.5f, -2.0f);

    const float fx0 = quadratic.Cost(MakeVector(p[0] + h, p[1]));
    const float fx1 = quadratic.Cost(MakeVector(p[0] - h, p[1]));
    const float fy0 = quadratic.Cost(MakeVector(p[0], p[1] + h));
    const float fy1 = quadratic.Cost(MakeVector(p[0], p[1] - h));

    const float fdGradX = (fx0 - fx1) / (2.0f * h);
    const float fdGradY = (fy0 - fy1) / (2.0f * h);

    const auto analytic = quadratic.Gradient(p);

    EXPECT_NEAR(analytic[0], fdGradX, 1e-3f);
    EXPECT_NEAR(analytic[1], fdGradY, 1e-3f);
}

TEST_F(TestGradientDescent, zero_gradient_leaves_parameters_unchanged)
{
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(0.5f, 10) };

    const auto initial = MakeVector(3.7f, -1.2f);
    const auto& result = optimizer.Minimize(initial, zeroGrad);

    EXPECT_NEAR(result.parameters[0], initial[0], math::Tolerance<float>());
    EXPECT_NEAR(result.parameters[1], initial[1], math::Tolerance<float>());
}

TEST_F(TestGradientDescent, deterministic_identical_inputs_produce_identical_outputs)
{
    optimization::GradientDescent<float, 2> optimizerA{ MakeParams(0.1f, 10) };
    optimization::GradientDescent<float, 2> optimizerB{ MakeParams(0.1f, 10) };

    const auto initial = MakeVector(2.0f, -3.0f);

    const auto& resultA = optimizerA.Minimize(initial, quadratic);
    const auto& resultB = optimizerB.Minimize(initial, quadratic);

    EXPECT_FLOAT_EQ(resultA.parameters[0], resultB.parameters[0]);
    EXPECT_FLOAT_EQ(resultA.parameters[1], resultB.parameters[1]);
    EXPECT_FLOAT_EQ(resultA.finalCost, resultB.finalCost);
    EXPECT_EQ(resultA.iterations, resultB.iterations);
}

TEST_F(TestGradientDescent, approaches_rosenbrock_minimum_within_budget)
{
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(1e-3f, 10000) };

    const auto& result = optimizer.Minimize(MakeVector(0.0f, 0.0f), rosenbrock);

    EXPECT_LT(result.finalCost, 0.1f);
}

TEST_F(TestGradientDescent, large_learning_rate_diverges_from_minimum)
{
    optimization::GradientDescent<float, 2> optimizer{ MakeParams(1.5f, 5) };

    const auto& result = optimizer.Minimize(MakeVector(1.0f, 1.0f), quadratic);

    const float initialCost = quadratic.Cost(MakeVector(1.0f, 1.0f));
    EXPECT_GT(result.finalCost, initialCost);
}
