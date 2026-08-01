// Copyright (c) 2024 Numerical Toolbox Contributors
// SPDX-License-Identifier: MIT

#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/nonlinear_control/FeedbackLinearization.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

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
