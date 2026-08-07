#include "numerical/filters/active/ExtendedKalmanFilter.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>

namespace
{
    using math::test::AreVectorsNear;

    using StateVec2 = math::Vector<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using StateMat2 = math::SquareMatrix<float, 2>;
    using MeasMat2x1 = math::Matrix<float, 1, 2>;
    using ControlVec1 = math::Vector<float, 1>;

    constexpr float kDt = 0.1f;

    StateVec2 LinearStateTransition(const StateVec2& x)
    {
        return StateVec2{
            { x.at(0, 0) + kDt * x.at(1, 0) },
            { x.at(1, 0) }
        };
    }

    StateMat2 LinearStateJacobian(const StateVec2& /*x*/)
    {
        return StateMat2{
            { 1.0f, kDt },
            { 0.0f, 1.0f }
        };
    }

    MeasVec1 LinearMeasurement(const StateVec2& x)
    {
        return MeasVec1{ { x.at(0, 0) } };
    }

    MeasMat2x1 LinearMeasurementJacobian(const StateVec2& /*x*/)
    {
        return MeasMat2x1{ { 1.0f, 0.0f } };
    }

    StateVec2 NonlinearStateTransition(const StateVec2& x)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * kDt },
            { vel - std::sin(pos) * kDt }
        };
    }

    StateMat2 NonlinearStateJacobian(const StateVec2& x)
    {
        float pos = x.at(0, 0);
        return StateMat2{
            { 1.0f, kDt },
            { -std::cos(pos) * kDt, 1.0f }
        };
    }

    StateVec2 StateTransitionWithControl(const StateVec2& x, const ControlVec1& u)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * kDt },
            { vel + u.at(0, 0) * kDt }
        };
    }

    StateMat2 StateJacobianWithControl(const StateVec2& /*x*/, const ControlVec1& /*u*/)
    {
        return StateMat2{
            { 1.0f, kDt },
            { 0.0f, 1.0f }
        };
    }

    class ExtendedKalmanFilterTest : public ::testing::Test
    {
    protected:
        using EkfType = filters::ExtendedKalmanFilter<float, 2, 1, 0>;
        using EkfWithControlType = filters::ExtendedKalmanFilter<float, 2, 1, 1>;

        StateVec2 initialState{};
        StateMat2 initialP{
            { 0.5f, 0.0f },
            { 0.0f, 0.5f }
        };

        std::optional<EkfType> ekf;
        std::optional<EkfWithControlType> ekfCtrl;

        void SetUpLinear()
        {
            ekf.emplace(initialState, initialP,
                LinearStateTransition, LinearStateJacobian,
                LinearMeasurement, LinearMeasurementJacobian);
            ekf->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
            ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.5f } });
        }

        void SetUpNonlinear()
        {
            ekf.emplace(initialState, initialP,
                NonlinearStateTransition, NonlinearStateJacobian,
                LinearMeasurement, LinearMeasurementJacobian);
        }

        void SetUpControl()
        {
            ekfCtrl.emplace(initialState, initialP,
                StateTransitionWithControl, StateJacobianWithControl,
                LinearMeasurement, LinearMeasurementJacobian);
            ekfCtrl->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
            ekfCtrl->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.5f } });
        }
    };

    class ExtendedKalmanFilter3StateTest : public ::testing::Test
    {
    protected:
        using StateVec3 = math::Vector<float, 3>;
        using StateMat3 = math::SquareMatrix<float, 3>;
        using MeasVec = math::Vector<float, 1>;
        using MeasCov = math::SquareMatrix<float, 1>;
        using MeasMat = math::Matrix<float, 1, 3>;
        using Ekf3 = filters::ExtendedKalmanFilter<float, 3, 1, 0>;

        std::optional<Ekf3> ekf;

        void SetUp() override
        {
            auto transition = [](const StateVec3& x) -> StateVec3
            {
                return StateVec3{
                    { x.at(0, 0) + x.at(1, 0) * kDt },
                    { x.at(1, 0) + x.at(2, 0) * kDt },
                    { x.at(2, 0) }
                };
            };

            auto jacobian = [](const StateVec3& /*x*/) -> StateMat3
            {
                return StateMat3{
                    { 1.0f, kDt, 0.0f },
                    { 0.0f, 1.0f, kDt },
                    { 0.0f, 0.0f, 1.0f }
                };
            };

            auto measurementFn = [](const StateVec3& x) -> MeasVec
            {
                return MeasVec{ { x.at(0, 0) } };
            };

            auto measurementJac = [](const StateVec3& /*x*/) -> MeasMat
            {
                return MeasMat{ { 1.0f, 0.0f, 0.0f } };
            };

            StateVec3 initialState3{};
            StateMat3 initialP3{
                { 1.0f, 0.0f, 0.0f },
                { 0.0f, 1.0f, 0.0f },
                { 0.0f, 0.0f, 1.0f }
            };

            ekf.emplace(initialState3, initialP3, transition, jacobian, measurementFn, measurementJac);
            ekf->SetProcessNoise(StateMat3{
                { 0.01f, 0.0f, 0.0f },
                { 0.0f, 0.01f, 0.0f },
                { 0.0f, 0.0f, 0.01f } });
            ekf->SetMeasurementNoise(MeasCov{ { 0.1f } });
        }
    };
}

TEST_F(ExtendedKalmanFilterTest, LinearSystemMatchesKalmanFilter)
{
    SetUpLinear();

    float truePos = 0.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * kDt;
        ekf->Predict();
        ekf->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ekf->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.4f);
}

TEST_F(ExtendedKalmanFilterTest, NonlinearPredictionUpdatesState)
{
    ekf.emplace(
        StateVec2{ { 0.1f }, { 0.0f } },
        StateMat2{
            { 0.5f, 0.0f },
            { 0.0f, 0.5f } },
        NonlinearStateTransition, NonlinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);
    ekf->SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });

    ekf->Predict();

    auto predicted = ekf->GetState();
    EXPECT_NEAR(predicted.at(0, 0), 0.1f, 0.05f);
    EXPECT_LT(predicted.at(1, 0), 0.0f);
}

TEST_F(ExtendedKalmanFilterTest, UpdateReducesCovariance)
{
    SetUpNonlinear();
    ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.5f } });

    auto covBefore = ekf->GetCovariance();
    ekf->Update(MeasVec1{ { 0.5f } });
    auto covAfter = ekf->GetCovariance();

    EXPECT_LT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(ExtendedKalmanFilterTest, NonlinearTrackingConverges)
{
    SetUpNonlinear();
    ekf->SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });
    ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.1f } });

    float truePos = 0.3f;
    float trueVel = 0.0f;

    for (int i = 0; i < 50; ++i)
    {
        float newVel = trueVel - std::sin(truePos) * kDt;
        float newPos = truePos + trueVel * kDt;
        truePos = newPos;
        trueVel = newVel;

        ekf->Predict();
        ekf->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ekf->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.1f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.2f);
}

TEST_F(ExtendedKalmanFilterTest, WithControlInput)
{
    SetUpControl();

    ControlVec1 controlInput{ { 1.0f } };
    float truePos = 0.0f;
    float trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * kDt;
        truePos += trueVel * kDt;

        ekfCtrl->Predict(controlInput);
        ekfCtrl->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ekfCtrl->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.4f);
}

TEST_F(ExtendedKalmanFilter3StateTest, ThreeStateSystemTracksAcceleration)
{
    float trueAcc = 0.5f;
    float trueVel3 = 0.0f;
    float truePos3 = 0.0f;

    for (int i = 0; i < 30; ++i)
    {
        trueVel3 += trueAcc * kDt;
        truePos3 += trueVel3 * kDt;

        ekf->Predict();
        ekf->Update(math::Vector<float, 1>{ { truePos3 } });
    }

    auto finalState3 = ekf->GetState();
    EXPECT_NEAR(finalState3.at(0, 0), truePos3, 0.15f);
    EXPECT_NEAR(finalState3.at(1, 0), trueVel3, 0.3f);
    EXPECT_NEAR(finalState3.at(2, 0), trueAcc, 0.5f);
}
