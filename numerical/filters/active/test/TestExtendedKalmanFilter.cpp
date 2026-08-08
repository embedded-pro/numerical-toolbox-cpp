#include "numerical/filters/active/ExtendedKalmanFilter.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using math::test::AreMatricesNear;
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

    StateMat2 LinearStateJacobian(const StateVec2&)
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

    MeasMat2x1 LinearMeasurementJacobian(const StateVec2&)
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

    StateMat2 StateJacobianWithControl(const StateVec2&, const ControlVec1&)
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
            ekf->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
            ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.5f } });
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

            auto jacobian = [](const StateVec3&) -> StateMat3
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

            auto measurementJac = [](const StateVec3&) -> MeasMat
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

    class ExtendedKalmanFilterConsistencyTest : public ::testing::Test
    {
    protected:
        using EkfType = filters::ExtendedKalmanFilter<float, 2, 1, 0>;
        using Cm1 = math::ConsistencyMetrics<float, 1>;
        using Cm2 = math::ConsistencyMetrics<float, 2>;
        using InnovVec = math::Vector<float, 1>;
        using InnovCov = math::SquareMatrix<float, 1>;

        StateMat2 Q{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
        math::SquareMatrix<float, 1> R{ { 0.1f } };

        std::optional<EkfType> ekf;

        void SetUp() override
        {
            ekf.emplace(StateVec2{}, StateMat2{ { 1.0f, 0.0f }, { 0.0f, 1.0f } },
                LinearStateTransition, LinearStateJacobian,
                LinearMeasurement, LinearMeasurementJacobian);
            ekf->SetProcessNoise(Q);
            ekf->SetMeasurementNoise(R);
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

TEST_F(ExtendedKalmanFilterTest, NonlinearPredictStepMatchesAnalyticTransition)
{
    StateVec2 x0{ { 0.3f }, { 0.5f } };
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    ekf.emplace(x0, p0,
        NonlinearStateTransition, NonlinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);
    ekf->SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });
    ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.5f } });

    ekf->Predict();

    float expectedPos = 0.3f + 0.5f * kDt;
    float expectedVel = 0.5f - std::sin(0.3f) * kDt;

    EXPECT_NEAR(ekf->GetState().at(0, 0), expectedPos, math::Tolerance<float>());
    EXPECT_NEAR(ekf->GetState().at(1, 0), expectedVel, math::Tolerance<float>());
}

TEST_F(ExtendedKalmanFilterTest, UpdateReducesCovariance)
{
    SetUpNonlinear();

    auto covBefore = ekf->GetCovariance();
    ekf->Update(MeasVec1{ { 0.5f } });
    auto covAfter = ekf->GetCovariance();

    EXPECT_LT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(ExtendedKalmanFilterTest, PredictGrowsCovarianceThenUpdateReducesIt)
{
    SetUpLinear();

    float p00Before = initialP.at(0, 0);

    ekf->Predict();
    float p00AfterPredict = ekf->GetCovariance().at(0, 0);
    EXPECT_GT(p00AfterPredict, p00Before);

    ekf->Update(MeasVec1{ { 0.1f } });
    EXPECT_LT(ekf->GetCovariance().at(0, 0), p00AfterPredict);
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

TEST_F(ExtendedKalmanFilterTest, ZeroInnovationLeavesStateUnchangedAndReducesCovariance)
{
    SetUpLinear();
    ekf->Predict();

    StateVec2 stateBefore = ekf->GetState();
    float p00Before = ekf->GetCovariance().at(0, 0);

    MeasVec1 exactMeas{ { ekf->GetState().at(0, 0) } };
    ekf->Update(exactMeas);

    EXPECT_NEAR(ekf->GetState().at(0, 0), stateBefore.at(0, 0), math::Tolerance<float>());
    EXPECT_LT(ekf->GetCovariance().at(0, 0), p00Before);
}

TEST_F(ExtendedKalmanFilterTest, CovarianceRemainsSymmetricAfterManySteps)
{
    SetUpLinear();

    float truePos = 0.0f;
    constexpr float trueVel = 0.3f;

    for (int i = 0; i < 50; ++i)
    {
        truePos += trueVel * kDt;
        ekf->Predict();
        ekf->Update(MeasVec1{ { truePos } });

        auto P = ekf->GetCovariance();
        EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
    }
}

TEST_F(ExtendedKalmanFilterTest, JosephFormKeepsCovariancePositiveSemiDefinite)
{
    ekf.emplace(initialState, StateMat2{ { 1000.0f, 0.0f }, { 0.0f, 1000.0f } },
        LinearStateTransition, LinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);
    ekf->SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });
    ekf->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.001f } });

    for (int i = 0; i < 50; ++i)
    {
        ekf->Predict();
        ekf->Update(MeasVec1{ { 1.0f } });

        auto P = ekf->GetCovariance();
        EXPECT_GE(P.at(0, 0), 0.0f);
        EXPECT_GE(P.at(1, 1), 0.0f);
        float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
        EXPECT_GE(det, -math::Tolerance<float>());
    }
}

TEST_F(ExtendedKalmanFilterTest, TwoIndependentInstancesProduceSameOutput)
{
    filters::ExtendedKalmanFilter<float, 2, 1, 0> filterA{
        initialState, initialP,
        LinearStateTransition, LinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian
    };
    filters::ExtendedKalmanFilter<float, 2, 1, 0> filterB{
        initialState, initialP,
        LinearStateTransition, LinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian
    };

    StateMat2 Q{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f }
    };
    math::SquareMatrix<float, 1> R{ { 0.5f } };

    for (auto* f : { &filterA, &filterB })
    {
        f->SetProcessNoise(Q);
        f->SetMeasurementNoise(R);
    }

    std::array<float, 5> measurements{ 0.1f, 0.25f, 0.42f, 0.61f, 0.83f };
    for (float z : measurements)
    {
        filterA.Predict();
        filterA.Update(MeasVec1{ { z } });
        filterB.Predict();
        filterB.Update(MeasVec1{ { z } });
    }

    EXPECT_FLOAT_EQ(filterA.GetState().at(0, 0), filterB.GetState().at(0, 0));
    EXPECT_FLOAT_EQ(filterA.GetState().at(1, 0), filterB.GetState().at(1, 0));
    EXPECT_FLOAT_EQ(filterA.GetCovariance().at(0, 0), filterB.GetCovariance().at(0, 0));
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

TEST_F(ExtendedKalmanFilterConsistencyTest, NeesIsNonNegativeAfterConvergence)
{
    constexpr float truePos = 2.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 40; ++i)
    {
        ekf->Predict();
        ekf->Update(MeasVec1{ { truePos + trueVel * static_cast<float>(i) * kDt } });
    }

    StateVec2 truth{ { truePos + trueVel * 40.0f * kDt }, { trueVel } };
    StateVec2 error;
    error.at(0, 0) = ekf->GetState().at(0, 0) - truth.at(0, 0);
    error.at(1, 0) = ekf->GetState().at(1, 0) - truth.at(1, 0);

    auto nees = Cm2::Nees(error, ekf->GetCovariance());
    ASSERT_TRUE(nees.has_value());
    EXPECT_GE(*nees, 0.0f);
}

TEST_F(ExtendedKalmanFilterConsistencyTest, NisIsNonNegativeAndFiniteOnEveryStep)
{
    constexpr float trueVel = 1.0f;
    constexpr int kSteps = 30;

    for (int i = 0; i < kSteps; ++i)
    {
        ekf->Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * kDt;
        MeasVec1 meas{ { measPos } };

        InnovVec innovation;
        innovation.at(0, 0) = measPos - ekf->GetState().at(0, 0);

        auto P = ekf->GetCovariance();
        InnovCov innovCov;
        innovCov.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, innovCov);
        ASSERT_TRUE(nis.has_value());
        EXPECT_GE(*nis, 0.0f);
        EXPECT_FALSE(std::isinf(*nis));
        EXPECT_FALSE(std::isnan(*nis));

        ekf->Update(meas);
    }
}

TEST_F(ExtendedKalmanFilterConsistencyTest, IsConsistentReturnsTrueForNeesInChiSquaredBand)
{
    StateVec2 error{ { 0.5f }, { 0.1f } };
    StateMat2 cov{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    auto nees = Cm2::Nees(error, cov);
    ASSERT_TRUE(nees.has_value());

    bool consistent = Cm2::IsConsistent(*nees);
    EXPECT_TRUE(consistent);
}

TEST_F(ExtendedKalmanFilterConsistencyTest, IsConsistentReturnsFalseForLargeNees)
{
    StateVec2 error{ { 100.0f }, { 100.0f } };
    StateMat2 cov{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    auto nees = Cm2::Nees(error, cov);
    ASSERT_TRUE(nees.has_value());

    bool consistent = Cm2::IsConsistent(*nees);
    EXPECT_FALSE(consistent);
}

TEST_F(ExtendedKalmanFilterConsistencyTest, TimeAveragedNisConsistentForNoiselessLinearSystem)
{
    constexpr int kSteps = 10;
    constexpr float trueVel = 0.2f;

    float nisSum = 0.0f;

    for (int i = 0; i < kSteps; ++i)
    {
        ekf->Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * kDt;
        InnovVec innovation;
        innovation.at(0, 0) = measPos - ekf->GetState().at(0, 0);

        auto P = ekf->GetCovariance();
        InnovCov S;
        S.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, S);
        ASSERT_TRUE(nis.has_value());
        nisSum += *nis;

        ekf->Update(MeasVec1{ { measPos } });
    }

    float avgNis = nisSum / static_cast<float>(kSteps);
    EXPECT_GE(avgNis, 0.0f);
}

TEST_F(ExtendedKalmanFilterConsistencyTest, IsTimeAveragedConsistentReturnsFalseForZeroSamples)
{
    EXPECT_FALSE(Cm1::IsTimeAveragedConsistent(1.0f, 0));
}

TEST_F(ExtendedKalmanFilterConsistencyTest, NeesReturnsNulloptForSingularCovariance)
{
    StateVec2 error{ { 1.0f }, { 0.0f } };
    StateMat2 singular{ { 0.0f, 0.0f }, { 0.0f, 0.0f } };

    auto nees = Cm2::Nees(error, singular);
    EXPECT_FALSE(nees.has_value());
}

TEST_F(ExtendedKalmanFilterConsistencyTest, NisReturnsNulloptForSingularInnovationCovariance)
{
    InnovVec innovation{ { 1.5f } };
    InnovCov singular{ { 0.0f } };

    auto nis = Cm1::Nis(innovation, singular);
    EXPECT_FALSE(nis.has_value());
}

TEST_F(ExtendedKalmanFilterConsistencyTest, NeesIsNonNegativeForAnyValidErrorAndCovariance)
{
    StateVec2 error{ { 2.0f }, { 1.0f } };
    StateMat2 cov{
        { 4.0f, 0.5f },
        { 0.5f, 2.0f }
    };

    auto nees = Cm2::Nees(error, cov);
    ASSERT_TRUE(nees.has_value());
    EXPECT_GE(*nees, 0.0f);
}
