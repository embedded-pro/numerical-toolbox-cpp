#include "numerical/filters/active/UnscentedKalmanFilter.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>

namespace
{
    using math::test::AreVectorsNear;

    using StateVec2 = math::Vector<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using StateMat2 = math::SquareMatrix<float, 2>;
    using MeasCov1 = math::SquareMatrix<float, 1>;
    using ControlVec1 = math::Vector<float, 1>;

    constexpr float kDt = 0.1f;

    StateVec2 LinearStateTransition(const StateVec2& x)
    {
        return StateVec2{
            { x.at(0, 0) + kDt * x.at(1, 0) },
            { x.at(1, 0) }
        };
    }

    MeasVec1 LinearMeasurement(const StateVec2& x)
    {
        return MeasVec1{ { x.at(0, 0) } };
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

    StateVec2 StateTransitionWithControl(const StateVec2& x, const ControlVec1& u)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * kDt },
            { vel + u.at(0, 0) * kDt }
        };
    }

    class UnscentedKalmanFilterTest : public ::testing::Test
    {
    protected:
        using UkfType = filters::UnscentedKalmanFilter<float, 2, 1, 0>;
        using UkfWithControlType = filters::UnscentedKalmanFilter<float, 2, 1, 1>;

        StateMat2 initialCovariance{
            { 0.5f, 0.0f },
            { 0.0f, 0.5f }
        };

        filters::UkfParameters params;
        std::optional<UkfType> ukf;
        std::optional<UkfWithControlType> ukfCtrl;

        void SetUp() override
        {
            params.alpha = 1e-1f;
            params.beta = 2.0f;
            params.kappa = 0.0f;
        }

        void BuildLinearUkf()
        {
            ukf.emplace(StateVec2{}, initialCovariance, LinearStateTransition, LinearMeasurement, params);
            ukf->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
            ukf->SetMeasurementNoise(MeasCov1{ { 0.5f } });
        }

        void BuildControlUkf()
        {
            ukfCtrl.emplace(StateVec2{}, initialCovariance, StateTransitionWithControl, LinearMeasurement, params);
            ukfCtrl->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
            ukfCtrl->SetMeasurementNoise(MeasCov1{ { 0.5f } });
        }
    };
}

TEST_F(UnscentedKalmanFilterTest, LinearSystemTracksConstantVelocity)
{
    BuildLinearUkf();

    float truePos = 0.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * kDt;
        ukf->Predict();
        ukf->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukf->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.5f);
}

TEST_F(UnscentedKalmanFilterTest, PredictGrowsCovariance)
{
    ukf.emplace(StateVec2{ { 0.0f }, { 0.1f } }, StateMat2{
        { 0.1f, 0.0f },
        { 0.0f, 0.1f } }, LinearStateTransition, LinearMeasurement, params);
    ukf->SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });

    auto covBefore = ukf->GetCovariance();
    ukf->Predict();
    auto covAfter = ukf->GetCovariance();

    EXPECT_GT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(UnscentedKalmanFilterTest, UpdateReducesCovariance)
{
    ukf.emplace(StateVec2{}, initialCovariance, LinearStateTransition, LinearMeasurement, params);
    ukf->SetMeasurementNoise(MeasCov1{ { 0.5f } });

    auto covBefore = ukf->GetCovariance();
    ukf->Update(MeasVec1{ { 0.5f } });
    auto covAfter = ukf->GetCovariance();

    EXPECT_LT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(UnscentedKalmanFilterTest, NonlinearTrackingConverges)
{
    ukf.emplace(StateVec2{}, initialCovariance, NonlinearStateTransition, LinearMeasurement, params);
    ukf->SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });
    ukf->SetMeasurementNoise(MeasCov1{ { 0.1f } });

    float truePos = 0.3f;
    float trueVel = 0.0f;

    for (int i = 0; i < 50; ++i)
    {
        float newVel = trueVel - std::sin(truePos) * kDt;
        float newPos = truePos + trueVel * kDt;
        truePos = newPos;
        trueVel = newVel;

        ukf->Predict();
        ukf->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukf->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.1f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.2f);
}

TEST_F(UnscentedKalmanFilterTest, WithControlInputTracksAcceleratingObject)
{
    BuildControlUkf();

    ControlVec1 controlInput{ { 1.0f } };
    float truePos = 0.0f;
    float trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * kDt;
        truePos += trueVel * kDt;

        ukfCtrl->Predict(controlInput);
        ukfCtrl->Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukfCtrl->GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.5f);
}

TEST_F(UnscentedKalmanFilterTest, ThreeStateConstantAccelerationConverges)
{
    using StateVec3 = math::Vector<float, 3>;
    using StateMat3 = math::SquareMatrix<float, 3>;
    using Ukf3 = filters::UnscentedKalmanFilter<float, 3, 1, 0>;

    auto transition = [](const StateVec3& x) -> StateVec3
    {
        return StateVec3{
            { x.at(0, 0) + x.at(1, 0) * kDt },
            { x.at(1, 0) + x.at(2, 0) * kDt },
            { x.at(2, 0) }
        };
    };

    auto measurementFn = [](const StateVec3& x) -> MeasVec1
    {
        return MeasVec1{ { x.at(0, 0) } };
    };

    StateMat3 initialP{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    Ukf3 ukf3(StateVec3{}, initialP, transition, measurementFn, params);
    ukf3.SetProcessNoise(StateMat3{
        { 0.01f, 0.0f, 0.0f },
        { 0.0f, 0.01f, 0.0f },
        { 0.0f, 0.0f, 0.01f } });
    ukf3.SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.1f } });

    constexpr float trueAcc = 0.5f;
    float trueVel3 = 0.0f;
    float truePos3 = 0.0f;

    for (int i = 0; i < 30; ++i)
    {
        trueVel3 += trueAcc * kDt;
        truePos3 += trueVel3 * kDt;

        ukf3.Predict();
        ukf3.Update(MeasVec1{ { truePos3 } });
    }

    auto finalState3 = ukf3.GetState();
    EXPECT_NEAR(finalState3.at(0, 0), truePos3, 0.15f);
    EXPECT_NEAR(finalState3.at(1, 0), trueVel3, 0.3f);
}
