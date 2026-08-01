#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/nonlinear_control/FeedbackLinearization.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <cmath>

namespace
{
    template<typename T, std::size_t Dim>
    class MockControlAffineModel
        : public nonlinear_control::ControlAffineModel<T, Dim>
    {
    public:
        using StateVector = math::Vector<T, Dim>;
        using DecouplingMatrix = math::SquareMatrix<T, Dim>;

        MOCK_METHOD(DecouplingMatrix, DecouplingMatrixAt, (const StateVector& x), (const, override));
        MOCK_METHOD(StateVector, DriftTerm, (const StateVector& x), (const, override));
    };

    template<typename T>
    class ManipulatorPlant
        : public nonlinear_control::ControlAffineModel<T, 2>
    {
    public:
        using StateVector = math::Vector<T, 2>;
        using DecouplingMatrix = math::SquareMatrix<T, 2>;

        [[nodiscard]] DecouplingMatrix DecouplingMatrixAt(const StateVector& q) const override
        {
            return InertiaMatrix(q);
        }

        [[nodiscard]] StateVector DriftTerm(const StateVector& q) const override
        {
            return GravityTerm(q);
        }

        void Step(const StateVector& u, T dt)
        {
            const StateVector qDdot{ SolveSpd(InertiaMatrix(q), u - GravityTerm(q)) };
            qDot = qDot + qDdot * dt;
            q = q + qDot * dt;
        }

        [[nodiscard]] StateVector Position() const
        {
            return q;
        }

        [[nodiscard]] StateVector Velocity() const
        {
            return qDot;
        }

    private:
        static DecouplingMatrix InertiaMatrix(const StateVector& q)
        {
            const T c{ std::cos(q.at(1, 0)) };
            return DecouplingMatrix{ static_cast<T>(2) + c, static_cast<T>(0.5),
                static_cast<T>(0.5), static_cast<T>(1) };
        }

        static StateVector GravityTerm(const StateVector& q)
        {
            return StateVector{ { std::sin(q.at(0, 0)) }, { std::sin(q.at(1, 0)) } };
        }

        static StateVector SolveSpd(const DecouplingMatrix& m, const StateVector& b)
        {
            const T det{ m.at(0, 0) * m.at(1, 1) - m.at(0, 1) * m.at(1, 0) };
            return StateVector{
                { (m.at(1, 1) * b.at(0, 0) - m.at(0, 1) * b.at(1, 0)) / det },
                { (m.at(0, 0) * b.at(1, 0) - m.at(1, 0) * b.at(0, 0)) / det }
            };
        }

        StateVector q{ { T{} }, { T{} } };
        StateVector qDot{ { T{} }, { T{} } };
    };

    class TestFeedbackLinearization
        : public ::testing::Test
    {
    protected:
        ::testing::StrictMock<MockControlAffineModel<float, 2>> model;

        math::SquareMatrix<float, 2> kp{ 100.0f, 0.0f, 0.0f, 100.0f };
        math::SquareMatrix<float, 2> kd{ 20.0f, 0.0f, 0.0f, 20.0f };

        nonlinear_control::FeedbackLinearization<float, 2> controller{ model, kp, kd };

        math::Vector<float, 2> zero{ { 0.0f }, { 0.0f } };
    };
}

TEST_F(TestFeedbackLinearization, cancels_to_integrator_chain)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> c{ { 3.0f }, { -2.0f } };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(a));

    const auto u{ controller.ComputeInput(zero, zero, zero, zero, c) };

    EXPECT_NEAR(u.at(0, 0), c.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), c.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestFeedbackLinearization, adds_drift_compensation)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> drift{ { 0.0f }, { 5.0f } };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(drift));

    const auto u{ controller.ComputeInput(zero, zero, zero, zero, zero) };

    EXPECT_NEAR(u.at(0, 0), drift.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), drift.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestFeedbackLinearization, pd_law_drives_position_error)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> x{ { 1.0f }, { 2.0f } };
    const math::Vector<float, 2> yd{ { 3.0f }, { 5.0f } };
    const math::Vector<float, 2> e{ yd - x };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(a));

    const auto u{ controller.ComputeInput(x, zero, yd, zero, zero) };
    const auto expected{ kp * e };

    EXPECT_NEAR(u.at(0, 0), expected.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), expected.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestFeedbackLinearization, pd_law_drives_velocity_error)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> xDot{ { 0.5f }, { -1.0f } };
    const math::Vector<float, 2> ydDot{ { 1.5f }, { 2.0f } };
    const math::Vector<float, 2> eDot{ ydDot - xDot };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(a));

    const auto u{ controller.ComputeInput(zero, xDot, zero, ydDot, zero) };
    const auto expected{ kd * eDot };

    EXPECT_NEAR(u.at(0, 0), expected.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), expected.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestFeedbackLinearization, decoupling_matrix_scales_virtual_input)
{
    const math::SquareMatrix<float, 2> B{ 2.0f, 0.0f, 0.0f, 3.0f };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> v{ { 1.0f }, { 1.0f } };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(B));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(a));

    const auto u{ controller.ComputeInput(zero, zero, zero, zero, v) };

    EXPECT_NEAR(u.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), 3.0f, math::Tolerance<float>());
}

TEST_F(TestFeedbackLinearization, closed_loop_error_decays)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> yd{ { 1.0f }, { 0.5f } };
    math::Vector<float, 2> x{ { 0.0f }, { 0.0f } };
    math::Vector<float, 2> xDot{ { 0.0f }, { 0.0f } };
    const float dt{ 0.01f };
    const int steps{ 300 };

    const math::Vector<float, 2> initError{ yd - x };
    const float prevErrorNorm{ initError.at(0, 0) * initError.at(0, 0) + initError.at(1, 0) * initError.at(1, 0) };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).Times(steps).WillRepeatedly(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).Times(steps).WillRepeatedly(::testing::Return(a));

    for (int i = 0; i < steps; ++i)
    {
        const auto u{ controller.ComputeInput(x, xDot, yd, zero, zero) };
        xDot = xDot + u * dt;
        x = x + xDot * dt;
    }

    const math::Vector<float, 2> finalError{ yd - x };
    const float finalNorm{ finalError.at(0, 0) * finalError.at(0, 0) + finalError.at(1, 0) * finalError.at(1, 0) };
    EXPECT_LT(finalNorm, prevErrorNorm);
}

TEST_F(TestFeedbackLinearization, reference_feedforward_used)
{
    const math::SquareMatrix<float, 2> identity{ math::SquareMatrix<float, 2>::Identity() };
    const math::Vector<float, 2> a{ { 0.0f }, { 0.0f } };
    const math::Vector<float, 2> c{ { 7.0f }, { -4.0f } };

    EXPECT_CALL(model, DecouplingMatrixAt(::testing::_)).WillOnce(::testing::Return(identity));
    EXPECT_CALL(model, DriftTerm(::testing::_)).WillOnce(::testing::Return(a));

    const auto u{ controller.ComputeInput(zero, zero, zero, zero, c) };

    EXPECT_NEAR(u.at(0, 0), c.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), c.at(1, 0), math::Tolerance<float>());
}

namespace
{
    class TestFeedbackLinearizationPlant
        : public ::testing::Test
    {
    protected:
        ManipulatorPlant<float> plant;

        math::SquareMatrix<float, 2> kp{ 100.0f, 0.0f, 0.0f, 100.0f };
        math::SquareMatrix<float, 2> kd{ 20.0f, 0.0f, 0.0f, 20.0f };

        nonlinear_control::FeedbackLinearization<float, 2> controller{ plant, kp, kd };

        math::Vector<float, 2> zero{ { 0.0f }, { 0.0f } };
    };
}

TEST_F(TestFeedbackLinearizationPlant, regulates_to_setpoint_despite_nonlinear_dynamics)
{
    const math::Vector<float, 2> yd{ { 1.0f }, { -0.5f } };
    const float dt{ 0.001f };
    const int steps{ 2000 };

    for (int i = 0; i < steps; ++i)
    {
        const auto u{ controller.ComputeInput(plant.Position(), plant.Velocity(), yd, zero, zero) };
        plant.Step(u, dt);
    }

    EXPECT_NEAR(plant.Position().at(0, 0), yd.at(0, 0), 1.0e-2f);
    EXPECT_NEAR(plant.Position().at(1, 0), yd.at(1, 0), 1.0e-2f);
    EXPECT_NEAR(plant.Velocity().at(0, 0), 0.0f, 1.0e-2f);
    EXPECT_NEAR(plant.Velocity().at(1, 0), 0.0f, 1.0e-2f);
}

TEST_F(TestFeedbackLinearizationPlant, tracks_sinusoidal_trajectory_with_feedforward)
{
    const float amplitude{ 0.3f };
    const float omega{ 3.0f };
    const float dt{ 0.001f };
    const int steps{ 3000 };

    for (int i = 0; i < steps; ++i)
    {
        const float t{ static_cast<float>(i) * dt };
        const float s{ std::sin(omega * t) };
        const float cc{ std::cos(omega * t) };
        const math::Vector<float, 2> yd{ { amplitude * s }, { amplitude * s } };
        const math::Vector<float, 2> ydDot{ { amplitude * omega * cc }, { amplitude * omega * cc } };
        const math::Vector<float, 2> ydDdot{ { -amplitude * omega * omega * s }, { -amplitude * omega * omega * s } };

        const auto u{ controller.ComputeInput(plant.Position(), plant.Velocity(), yd, ydDot, ydDdot) };
        plant.Step(u, dt);
    }

    const float tFinal{ static_cast<float>(steps) * dt };
    const float ydFinal{ amplitude * std::sin(omega * tFinal) };

    EXPECT_NEAR(plant.Position().at(0, 0), ydFinal, 1.0e-2f);
    EXPECT_NEAR(plant.Position().at(1, 0), ydFinal, 1.0e-2f);
}
