#include "numerical/solvers/SymplecticIntegrators.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>

namespace
{
    using Vector1 = math::Vector<float, 1>;
    using State = solvers::SecondOrderState<float, 1>;

    MATCHER_P(ScalarNear, expected, "")
    {
        return std::abs(arg[0] - expected) <= 1e-6f;
    }

    class HarmonicOscillator
        : public solvers::SecondOrderSystem<float, 1, 1>
    {
    public:
        explicit HarmonicOscillator(float omega)
            : omega{ omega }
        {}

        Vector1 Acceleration(const Vector1& q, const Vector1&, const Vector1& u) override
        {
            return Vector1{ -omega * omega * q[0] + u[0] };
        }

    private:
        float omega;
    };

    class MockSecondOrderSystem
        : public solvers::SecondOrderSystem<float, 1, 1>
    {
    public:
        MOCK_METHOD(Vector1, Acceleration, (const Vector1& q, const Vector1& qDot, const Vector1& u), (override));
    };

    class TestSymplecticIntegrators
        : public ::testing::Test
    {
    protected:
        HarmonicOscillator oscillator{ 1.0f };
        testing::StrictMock<MockSecondOrderSystem> system;
        solvers::SemiImplicitEuler<float, 1, 1> euler{ oscillator, 0.01f };
        solvers::VelocityVerlet<float, 1, 1> verlet{ oscillator, 0.01f };
        const Vector1 noInput{ 0.0f };

        static float Energy(const State& x)
        {
            return 0.5f * (x.qDot[0] * x.qDot[0] + x.q[0] * x.q[0]);
        }

        template<class Integrator>
        float MaxRelativeEnergyError(Integrator& integrator, int steps) const
        {
            State x{ Vector1{ 1.0f }, Vector1{ 0.0f } };
            const float initialEnergy{ Energy(x) };
            float maxError{ 0.0f };

            for (int i = 0; i < steps; ++i)
            {
                x = integrator.Step(x, noInput);
                maxError = std::max(maxError, std::abs(Energy(x) - initialEnergy) / initialEnergy);
            }

            return maxError;
        }

        float VerletPositionErrorAtOneSecond(float h)
        {
            solvers::VelocityVerlet<float, 1, 1> integrator{ oscillator, h };
            State x{ Vector1{ 1.0f }, Vector1{ 0.0f } };
            const int steps{ static_cast<int>(std::lround(1.0f / h)) };

            for (int i = 0; i < steps; ++i)
                x = integrator.Step(x, noInput);

            return std::abs(x.q[0] - std::cos(1.0f));
        }
    };
}

TEST_F(TestSymplecticIntegrators, semi_implicit_euler_updates_position_with_new_velocity)
{
    solvers::SemiImplicitEuler<float, 1, 1> integrator{ system, 0.1f };
    EXPECT_CALL(system, Acceleration(ScalarNear(1.0f), ScalarNear(0.0f), ScalarNear(0.0f))).WillOnce(testing::Return(Vector1{ 2.0f }));

    const State next{ integrator.Step(State{ Vector1{ 1.0f }, Vector1{ 0.0f } }, noInput) };

    EXPECT_NEAR(next.q[0], 1.02f, 1e-6f);
    EXPECT_NEAR(next.qDot[0], 0.2f, 1e-6f);
}

TEST_F(TestSymplecticIntegrators, velocity_verlet_evaluates_at_half_step_velocity)
{
    solvers::VelocityVerlet<float, 1, 1> integrator{ system, 0.1f };
    testing::InSequence sequence;
    EXPECT_CALL(system, Acceleration(ScalarNear(1.0f), ScalarNear(0.0f), ScalarNear(0.0f))).WillOnce(testing::Return(Vector1{ 2.0f }));
    EXPECT_CALL(system, Acceleration(ScalarNear(1.01f), ScalarNear(0.1f), ScalarNear(0.0f))).WillOnce(testing::Return(Vector1{ 2.0f }));

    const State next{ integrator.Step(State{ Vector1{ 1.0f }, Vector1{ 0.0f } }, noInput) };

    EXPECT_NEAR(next.q[0], 1.01f, 1e-6f);
    EXPECT_NEAR(next.qDot[0], 0.2f, 1e-6f);
}

TEST_F(TestSymplecticIntegrators, velocity_verlet_is_exact_for_constant_acceleration)
{
    HarmonicOscillator free{ 0.0f };
    solvers::VelocityVerlet<float, 1, 1> integrator{ free, 0.01f };
    const Vector1 thrust{ 3.0f };
    State x{};

    for (int i = 0; i < 100; ++i)
        x = integrator.Step(x, thrust);

    EXPECT_NEAR(x.q[0], 1.5f, 1e-4f);
    EXPECT_NEAR(x.qDot[0], 3.0f, 1e-4f);
}

TEST_F(TestSymplecticIntegrators, energy_error_stays_bounded_over_long_runs)
{
    EXPECT_LT(MaxRelativeEnergyError(euler, 100000), 0.006f);
    EXPECT_LT(MaxRelativeEnergyError(verlet, 100000), 1e-4f);
}

TEST_F(TestSymplecticIntegrators, verlet_is_second_order)
{
    const float ratio{ VerletPositionErrorAtOneSecond(0.1f) / VerletPositionErrorAtOneSecond(0.05f) };

    EXPECT_NEAR(ratio, 4.0f, 0.4f);
}
