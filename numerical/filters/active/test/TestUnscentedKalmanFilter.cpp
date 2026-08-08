#include "numerical/filters/active/UnscentedKalmanFilter.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace
{
    using math::test::AreMatricesNear;
    using math::test::AreVectorsNear;

    using StateVec2 = math::Vector<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using StateMat2 = math::SquareMatrix<float, 2>;
    using MeasCov1 = math::SquareMatrix<float, 1>;
    using ControlVec1 = math::Vector<float, 1>;

    constexpr float kDt = 0.1f;

    StateVec2 IdentityTransition(const StateVec2& x)
    {
        return x;
    }

    StateVec2 ConstantVelocityTransition(const StateVec2& x)
    {
        return StateVec2{
            { x.at(0, 0) + kDt * x.at(1, 0) },
            { x.at(1, 0) }
        };
    }

    MeasVec1 PositionMeasurement(const StateVec2& x)
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

    filters::UkfParameters DefaultParams()
    {
        filters::UkfParameters p;
        p.alpha = 1e-1f;
        p.beta = 2.0f;
        p.kappa = 0.0f;
        return p;
    }

    class UnscentedKalmanFilterTest : public ::testing::Test
    {
    protected:
        using UkfType = filters::UnscentedKalmanFilter<float, 2, 1, 0>;
        using UkfWithControlType = filters::UnscentedKalmanFilter<float, 2, 1, 1>;

        filters::UkfParameters params{ DefaultParams() };

        StateMat2 InitP()
        {
            return StateMat2{
                { 0.5f, 0.0f },
                { 0.0f, 0.5f }
            };
        }
    };

    class UnscentedKalmanFilterConsistencyTest : public ::testing::Test
    {
    protected:
        using UkfType = filters::UnscentedKalmanFilter<float, 2, 1, 0>;
        using Cm1 = math::ConsistencyMetrics<float, 1>;
        using Cm2 = math::ConsistencyMetrics<float, 2>;
        using InnovVec = math::Vector<float, 1>;
        using InnovCov = math::SquareMatrix<float, 1>;

        StateMat2 Q{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
        MeasCov1 R{ { 0.1f } };

        std::optional<UkfType> ukf;

        void SetUp() override
        {
            ukf.emplace(StateVec2{}, StateMat2{
                { 1.0f, 0.0f },
                { 0.0f, 1.0f } },
                ConstantVelocityTransition, PositionMeasurement,
                DefaultParams());
            ukf->SetProcessNoise(Q);
            ukf->SetMeasurementNoise(R);
        }
    };
}

TEST_F(UnscentedKalmanFilterTest, IdentityTransitionPreservesMeanAndAddsQ)
{
    StateVec2 x0{ { 1.0f }, { 0.0f } };
    StateMat2 p0{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    StateMat2 Q{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f }
    };

    UkfType ukf{ x0, p0, IdentityTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(Q);

    ukf.Predict();

    EXPECT_NEAR(ukf.GetState().at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(ukf.GetState().at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(ukf.GetCovariance().at(0, 0), 1.01f, 1e-5f);
    EXPECT_NEAR(ukf.GetCovariance().at(1, 1), 1.01f, 1e-5f);
    EXPECT_NEAR(ukf.GetCovariance().at(0, 1), 0.0f, math::Tolerance<float>());
}

TEST_F(UnscentedKalmanFilterTest, UpdateProducesReferenceStateAndCovariance)
{
    StateVec2 x0{ { 1.5f }, { 0.0f } };
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 1.0f }
    };

    UkfType ukf{ x0, p0, IdentityTransition, PositionMeasurement, params };
    ukf.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    ukf.Update(MeasVec1{ { 2.0f } });

    EXPECT_NEAR(ukf.GetState().at(0, 0), 1.75f, math::Tolerance<float>());
    EXPECT_NEAR(ukf.GetState().at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(ukf.GetCovariance().at(0, 0), 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(ukf.GetCovariance().at(1, 1), 1.0f, math::Tolerance<float>());
}

TEST_F(UnscentedKalmanFilterTest, PredictGrowsCovariance)
{
    StateVec2 x0{ { 0.0f }, { 0.1f } };
    StateMat2 p0{
        { 0.1f, 0.0f },
        { 0.0f, 0.1f }
    };

    UkfType ukf{ x0, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });

    auto covBefore = ukf.GetCovariance();
    ukf.Predict();
    auto covAfter = ukf.GetCovariance();

    EXPECT_GT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(UnscentedKalmanFilterTest, UpdateReducesCovariance)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    UkfType ukf{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    auto covBefore = ukf.GetCovariance();
    ukf.Update(MeasVec1{ { 0.5f } });
    auto covAfter = ukf.GetCovariance();

    EXPECT_LT(covAfter.at(0, 0), covBefore.at(0, 0));
}

TEST_F(UnscentedKalmanFilterTest, ZeroInnovationLeavesStateUnchangedAndReducesCovariance)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    UkfType ukf{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    ukf.Predict();
    StateVec2 stateBefore = ukf.GetState();
    float p00Before = ukf.GetCovariance().at(0, 0);

    MeasVec1 exactMeas{ { ukf.GetState().at(0, 0) } };
    ukf.Update(exactMeas);

    EXPECT_NEAR(ukf.GetState().at(0, 0), stateBefore.at(0, 0), math::Tolerance<float>());
    EXPECT_LT(ukf.GetCovariance().at(0, 0), p00Before);
}

TEST_F(UnscentedKalmanFilterTest, CovarianceRemainsSymmetricAfterManySteps)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    UkfType ukf{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });
    ukf.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    float truePos = 0.0f;
    constexpr float trueVel = 0.3f;

    for (int i = 0; i < 50; ++i)
    {
        truePos += trueVel * kDt;
        ukf.Predict();
        ukf.Update(MeasVec1{ { truePos } });

        auto P = ukf.GetCovariance();
        EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
    }
}

TEST_F(UnscentedKalmanFilterTest, CovarianceRemainsPositiveSemiDefiniteOnLongHorizon)
{
    StateVec2 x0{ { 0.0f }, { 0.0f } };
    StateMat2 p0{
        { 1000.0f, 0.0f },
        { 0.0f, 1000.0f }
    };

    UkfType ukf{ x0, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });
    ukf.SetMeasurementNoise(MeasCov1{ { 0.001f } });

    for (int i = 0; i < 50; ++i)
    {
        ukf.Predict();
        ukf.Update(MeasVec1{ { 1.0f } });

        auto P = ukf.GetCovariance();
        EXPECT_GE(P.at(0, 0), 0.0f);
        EXPECT_GE(P.at(1, 1), 0.0f);
        float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
        EXPECT_GE(det, -math::Tolerance<float>());
    }
}

TEST_F(UnscentedKalmanFilterTest, TwoIndependentInstancesProduceSameOutput)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    StateMat2 Q{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f }
    };
    MeasCov1 R{ { 0.5f } };

    UkfType filterA{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };
    UkfType filterB{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };

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

TEST_F(UnscentedKalmanFilterTest, LinearSystemTracksConstantVelocity)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };

    UkfType ukf{ StateVec2{}, p0, ConstantVelocityTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });
    ukf.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    float truePos = 0.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * kDt;
        ukf.Predict();
        ukf.Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukf.GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.5f);
}

TEST_F(UnscentedKalmanFilterTest, NonlinearTrackingConverges)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };

    UkfType ukf{ StateVec2{}, p0, NonlinearStateTransition, PositionMeasurement, params };
    ukf.SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });
    ukf.SetMeasurementNoise(MeasCov1{ { 0.1f } });

    float truePos = 0.3f;
    float trueVel = 0.0f;

    for (int i = 0; i < 50; ++i)
    {
        float newVel = trueVel - std::sin(truePos) * kDt;
        float newPos = truePos + trueVel * kDt;
        truePos = newPos;
        trueVel = newVel;

        ukf.Predict();
        ukf.Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukf.GetState();
    EXPECT_NEAR(finalState.at(0, 0), truePos, 0.1f);
    EXPECT_NEAR(finalState.at(1, 0), trueVel, 0.2f);
}

TEST_F(UnscentedKalmanFilterTest, WithControlInputTracksAcceleratingObject)
{
    StateMat2 p0{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };

    UkfWithControlType ukfCtrl{ StateVec2{}, p0, StateTransitionWithControl, PositionMeasurement, params };
    ukfCtrl.SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });
    ukfCtrl.SetMeasurementNoise(MeasCov1{ { 0.5f } });

    ControlVec1 controlInput{ { 1.0f } };
    float truePos = 0.0f;
    float trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * kDt;
        truePos += trueVel * kDt;

        ukfCtrl.Predict(controlInput);
        ukfCtrl.Update(MeasVec1{ { truePos } });
    }

    auto finalState = ukfCtrl.GetState();
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

    Ukf3 ukf3(StateVec3{}, initialP, transition, measurementFn, DefaultParams());
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

TEST_F(UnscentedKalmanFilterConsistencyTest, NeesIsNonNegativeAfterConvergence)
{
    constexpr float truePos = 2.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 40; ++i)
    {
        ukf->Predict();
        ukf->Update(MeasVec1{ { truePos + trueVel * static_cast<float>(i) * kDt } });
    }

    StateVec2 truth{ { truePos + trueVel * 40.0f * kDt }, { trueVel } };
    StateVec2 error;
    error.at(0, 0) = ukf->GetState().at(0, 0) - truth.at(0, 0);
    error.at(1, 0) = ukf->GetState().at(1, 0) - truth.at(1, 0);

    auto nees = Cm2::Nees(error, ukf->GetCovariance());
    ASSERT_TRUE(nees.has_value());
    EXPECT_GE(*nees, 0.0f);
}

TEST_F(UnscentedKalmanFilterConsistencyTest, NisIsNonNegativeAndFiniteOnEveryStep)
{
    constexpr float trueVel = 1.0f;
    constexpr int kSteps = 30;

    for (int i = 0; i < kSteps; ++i)
    {
        ukf->Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * kDt;
        MeasVec1 meas{ { measPos } };

        InnovVec innovation;
        innovation.at(0, 0) = measPos - ukf->GetState().at(0, 0);

        auto P = ukf->GetCovariance();
        InnovCov innovCov;
        innovCov.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, innovCov);
        ASSERT_TRUE(nis.has_value());
        EXPECT_GE(*nis, 0.0f);
        EXPECT_FALSE(std::isinf(*nis));
        EXPECT_FALSE(std::isnan(*nis));

        ukf->Update(meas);
    }
}
