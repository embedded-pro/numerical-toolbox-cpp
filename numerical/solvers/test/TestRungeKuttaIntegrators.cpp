#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/DormandPrince45.hpp"
#include "numerical/solvers/RungeKuttaIntegrators.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <cmath>

namespace
{
    using State1 = math::Vector<float, 1>;
    using State2 = math::Vector<float, 2>;

    class ScalarDecay
        : public solvers::OdeSystem<float, 1, 0>
    {
    public:
        explicit ScalarDecay(float lambda)
            : lambda_{ lambda }
        {}

        State1 Derivative(const State1& x, float) override
        {
            return State1{ { -lambda_ * x.at(0, 0) } };
        }

    private:
        float lambda_;
    };

    class HarmonicOscillator
        : public solvers::OdeSystem<float, 2, 0>
    {
    public:
        explicit HarmonicOscillator(float omega)
            : omega_{ omega }
        {}

        State2 Derivative(const State2& x, float) override
        {
            return State2{ { x.at(1, 0) }, { -omega_ * omega_ * x.at(0, 0) } };
        }

    private:
        float omega_;
    };

    class ZeroDerivative
        : public solvers::OdeSystem<float, 1, 0>
    {
    public:
        State1 Derivative(const State1&, float) override
        {
            return State1{ { 0.0f } };
        }
    };

    class MockOdeSystem
        : public solvers::OdeSystem<float, 1, 0>
    {
    public:
        MOCK_METHOD(State1, Derivative, (const State1& x, float t), (override));
    };

    class TestRungeKutta
        : public ::testing::Test
    {
    protected:
        ScalarDecay decay{ 1.0f };
        HarmonicOscillator oscillator{ 2.0f };
        ZeroDerivative zeroSys;
        testing::StrictMock<MockOdeSystem> systemMock;

        solvers::RungeKutta4<float, 1, 0> rk4{ decay, 0.01f };
        solvers::DormandPrince45<float, 1, 0> dp45{ decay, 1e-6f, 1e-6f };
    };
}

TEST_F(TestRungeKutta, rk4_matches_exponential_decay)
{
    State1 x{ { 1.0f } };
    float t{ 0.0f };
    for (int i{ 0 }; i < 100; ++i)
    {
        x = rk4.Step(x, t);
        t += 0.01f;
    }
    EXPECT_NEAR(x.at(0, 0), std::exp(-1.0f), 1e-4f);
}

TEST_F(TestRungeKutta, rk4_is_fourth_order)
{
    auto integrate = [&](float h, int steps) -> float
    {
        solvers::RungeKutta4<float, 1, 0> integrator{ decay, h };
        State1 x{ { 1.0f } };
        float t{ 0.0f };
        for (int i{ 0 }; i < steps; ++i)
        {
            x = integrator.Step(x, t);
            t += h;
        }
        return x.at(0, 0);
    };

    float exact{ std::exp(-1.0f) };
    float errCoarse{ std::abs(integrate(0.1f, 10) - exact) };
    float errFine{ std::abs(integrate(0.05f, 20) - exact) };

    EXPECT_GT(errCoarse / errFine, 8.0f);
}

TEST_F(TestRungeKutta, rk4_conserves_oscillator_energy)
{
    solvers::RungeKutta4<float, 2, 0> rk4Osc{ oscillator, 0.001f };
    float omega{ 2.0f };
    State2 x{ { 1.0f }, { 0.0f } };
    float energyInitial{ 0.5f * (x.at(1, 0) * x.at(1, 0) + omega * omega * x.at(0, 0) * x.at(0, 0)) };
    float t{ 0.0f };
    for (int i{ 0 }; i < 1000; ++i)
    {
        x = rk4Osc.Step(x, t);
        t += 0.001f;
    }
    float energyFinal{ 0.5f * (x.at(1, 0) * x.at(1, 0) + omega * omega * x.at(0, 0) * x.at(0, 0)) };
    EXPECT_NEAR(energyFinal, energyInitial, 1e-3f);
}

TEST_F(TestRungeKutta, dp45_keeps_error_below_tolerance)
{
    State1 x{ { 1.0f } };
    float t{ 0.0f };
    float h{ 0.1f };
    for (int i{ 0 }; i < 20; ++i)
    {
        auto result{ dp45.Step(x, t, h) };
        if (result.accepted)
        {
            float exact{ std::exp(-(t + result.hUsed)) };
            EXPECT_NEAR(result.xNext.at(0, 0), exact, 1e-3f);
            x = result.xNext;
            t += result.hUsed;
        }
        h = result.hNext;
    }
}

TEST_F(TestRungeKutta, dp45_rejects_oversized_step)
{
    solvers::DormandPrince45<float, 1, 0> dp{ decay, 1e-10f, 1e-10f };
    dp.SetStepBounds(1e-12f, 100.0f);
    State1 x{ { 1.0f } };
    auto result{ dp.Step(x, 0.0f, 10.0f) };
    EXPECT_FALSE(result.accepted);
    EXPECT_LT(result.hNext, result.hUsed);
}

TEST_F(TestRungeKutta, dp45_grows_step_in_smooth_region)
{
    State1 x{ { 1.0f } };
    float t{ 0.0f };
    float h{ 0.01f };
    float hPrev{ h };
    bool grew{ false };
    dp45.SetStepBounds(1e-10f, 10.0f);
    for (int i{ 0 }; i < 50; ++i)
    {
        auto result{ dp45.Step(x, t, h) };
        if (result.accepted)
        {
            if (result.hNext > hPrev)
                grew = true;
            hPrev = result.hNext;
            x = result.xNext;
            t += result.hUsed;
        }
        h = result.hNext;
    }
    EXPECT_TRUE(grew);
}

TEST_F(TestRungeKutta, dp45_fsal_uses_six_evaluations)
{
    State1 constVal{ { 0.0f } };
    EXPECT_CALL(systemMock, Derivative(testing::_, testing::_))
        .WillRepeatedly(testing::Return(constVal));

    solvers::DormandPrince45<float, 1, 0> dp{ systemMock, 1e-6f, 1e-6f };
    State1 x{ { 1.0f } };

    auto first{ dp.Step(x, 0.0f, 0.01f) };
    ASSERT_TRUE(first.accepted);
    x = first.xNext;

    testing::Mock::VerifyAndClearExpectations(&systemMock);

    EXPECT_CALL(systemMock, Derivative(testing::_, testing::_))
        .Times(6)
        .WillRepeatedly(testing::Return(constVal));

    auto second{ dp.Step(x, 0.01f, first.hNext) };
    EXPECT_TRUE(second.accepted);
}

TEST_F(TestRungeKutta, zero_derivative_is_fixed_point)
{
    solvers::RungeKutta4<float, 1, 0> rk4Zero{ zeroSys, 0.1f };
    State1 x{ { 3.14f } };
    auto xNext{ rk4Zero.Step(x, 0.0f) };
    EXPECT_NEAR(xNext.at(0, 0), 3.14f, math::Tolerance<float>());

    solvers::DormandPrince45<float, 1, 0> dp45Zero{ zeroSys, 1e-6f, 1e-6f };
    auto result{ dp45Zero.Step(x, 0.0f, 0.1f) };
    if (result.accepted)
        EXPECT_NEAR(result.xNext.at(0, 0), 3.14f, math::Tolerance<float>());
}
