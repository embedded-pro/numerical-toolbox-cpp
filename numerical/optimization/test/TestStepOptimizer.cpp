#include "numerical/math/Tolerance.hpp"
#include "numerical/optimization/Adam.hpp"
#include "numerical/optimization/Sgd.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using Vector2f = math::Vector<float, 2>;

    Vector2f MakeVec(float x, float y)
    {
        Vector2f v{};
        v[0] = x;
        v[1] = y;
        return v;
    }

    class TestStepOptimizer
        : public ::testing::Test
    {};
}

TEST_F(TestStepOptimizer, single_step_against_hand_computed_values)
{
    optimization::Sgd<float, 2> sgd{ { 0.1f, 0.0f, false } };
    auto theta = MakeVec(1.0f, 2.0f);
    sgd.Step(theta, MakeVec(0.5f, 1.0f));
    EXPECT_NEAR(theta[0], 0.95f, 1e-5f);
    EXPECT_NEAR(theta[1], 1.9f, 1e-5f);

    optimization::Sgd<float, 2> nsgd{ { 0.1f, 0.9f, true } };
    auto thetaN = MakeVec(1.0f, 2.0f);
    nsgd.Step(thetaN, MakeVec(0.5f, 1.0f));
    EXPECT_NEAR(thetaN[0], 0.905f, 1e-5f);
    EXPECT_NEAR(thetaN[1], 1.81f, 1e-5f);
}

TEST_F(TestStepOptimizer, convergence_on_quadratic)
{
    optimization::Sgd<float, 2> sgd{ { 0.1f, 0.0f, false } };
    auto theta = MakeVec(1.0f, 1.0f);

    for (int i = 0; i < 100; ++i)
        sgd.Step(theta, theta);

    EXPECT_NEAR(theta[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(theta[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestStepOptimizer, adam_bias_correction_at_t1)
{
    optimization::Adam<float, 2> adam{ { 0.001f, 0.9f, 0.999f, 1e-8f } };
    auto theta = MakeVec(0.0f, 0.0f);
    adam.Step(theta, MakeVec(1.0f, 1.0f));

    const float beta1Correction = 1.0f - 0.9f;
    const float beta2Correction = 1.0f - 0.999f;
    const float mHat = (0.1f) / beta1Correction;
    const float vHat = (0.001f) / beta2Correction;
    const float expected = -0.001f * mHat / (std::sqrt(vHat) + 1e-8f);

    EXPECT_NEAR(theta[0], expected, 1e-5f);
    EXPECT_NEAR(theta[1], expected, 1e-5f);
}

TEST_F(TestStepOptimizer, reset_restores_initial_state)
{
    optimization::Sgd<float, 2> sgd{ { 0.1f, 0.9f, false } };
    auto theta1 = MakeVec(1.0f, 2.0f);
    sgd.Step(theta1, MakeVec(0.5f, 1.0f));

    sgd.Reset();

    auto theta2 = MakeVec(1.0f, 2.0f);
    sgd.Step(theta2, MakeVec(0.5f, 1.0f));

    EXPECT_NEAR(theta1[0], theta2[0], 1e-6f);
    EXPECT_NEAR(theta1[1], theta2[1], 1e-6f);

    optimization::Adam<float, 2> adam{ { 0.01f } };
    auto theta3 = MakeVec(1.0f, 2.0f);
    adam.Step(theta3, MakeVec(0.5f, 1.0f));
    adam.Step(theta3, MakeVec(0.5f, 1.0f));

    adam.Reset();

    auto theta4 = MakeVec(1.0f, 2.0f);
    adam.Step(theta4, MakeVec(0.5f, 1.0f));

    EXPECT_NEAR(theta4[0], 0.99f, 1e-5f);
    EXPECT_NEAR(theta4[1], 1.99f, 1e-5f);
}
