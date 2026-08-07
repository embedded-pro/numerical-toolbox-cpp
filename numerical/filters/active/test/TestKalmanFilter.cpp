#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/test_doubles/MatrixTestSupport.hpp"
#include <gtest/gtest.h>

namespace
{
    using math::test::AreMatricesNear;
    using math::test::AreVectorsNear;

    using StateVec2 = math::Vector<float, 2>;
    using StateMat2 = math::SquareMatrix<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using MeasMat1x2 = math::Matrix<float, 1, 2>;
    using MeasCov1 = math::SquareMatrix<float, 1>;

    constexpr float kTolerance = 1e-3f;
    constexpr float kDt = 0.1f;

    class KalmanFilterTest : public ::testing::Test
    {
    protected:
        using FilterType = filters::KalmanFilter<float, 2, 1>;

        StateMat2 initialCovariance{
            { 0.5f, 0.0f },
            { 0.0f, 0.5f }
        };

        std::optional<FilterType> filter;

        void SetUp() override
        {
            filter.emplace(StateVec2{}, initialCovariance);
        }

        void ConfigureConstantVelocityModel()
        {
            filter->SetStateTransition(StateMat2{
                { 1.0f, kDt },
                { 0.0f, 1.0f } });
            filter->SetMeasurementMatrix(MeasMat1x2{ { 1.0f, 0.0f } });
            filter->SetMeasurementNoise(MeasCov1{ { 0.5f } });
            filter->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
        }
    };

    class KalmanFilter3StateTest : public ::testing::Test
    {
    protected:
        using Filter3 = filters::KalmanFilter<float, 3, 1, 0>;
        using StateVec3 = math::Vector<float, 3>;
        using StateMat3 = math::SquareMatrix<float, 3>;
        using MeasMat1x3 = math::Matrix<float, 1, 3>;

        std::optional<Filter3> filter;

        void SetUp() override
        {
            StateVec3 initialState{};
            StateMat3 initialP{
                { 1.0f, 0.0f, 0.0f },
                { 0.0f, 1.0f, 0.0f },
                { 0.0f, 0.0f, 1.0f }
            };
            filter.emplace(initialState, initialP);
            filter->SetStateTransition(StateMat3{
                { 1.0f, kDt, 0.5f * kDt * kDt },
                { 0.0f, 1.0f, kDt },
                { 0.0f, 0.0f, 1.0f } });
            filter->SetMeasurementMatrix(MeasMat1x3{ { 1.0f, 0.0f, 0.0f } });
            filter->SetMeasurementNoise(math::SquareMatrix<float, 1>{ { 0.1f } });
            filter->SetProcessNoise(StateMat3{
                { 0.01f, 0.0f, 0.0f },
                { 0.0f, 0.01f, 0.0f },
                { 0.0f, 0.0f, 0.01f } });
        }
    };

    class KalmanFilter4StateTest : public ::testing::Test
    {
    protected:
        using Filter4 = filters::KalmanFilter<float, 4, 2, 0>;
        using StateVec4 = math::Vector<float, 4>;
        using StateMat4 = math::SquareMatrix<float, 4>;
        using MeasMat2x4 = math::Matrix<float, 2, 4>;

        std::optional<Filter4> filter;

        void SetUp() override
        {
            StateVec4 initialState{};
            StateMat4 initialP{
                { 1.0f, 0.0f, 0.0f, 0.0f },
                { 0.0f, 1.0f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 1.0f, 0.0f },
                { 0.0f, 0.0f, 0.0f, 1.0f }
            };
            filter.emplace(initialState, initialP);
            filter->SetStateTransition(StateMat4{
                { 1.0f, kDt, 0.0f, 0.0f },
                { 0.0f, 1.0f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 1.0f, kDt },
                { 0.0f, 0.0f, 0.0f, 1.0f } });
            filter->SetMeasurementMatrix(MeasMat2x4{
                { 1.0f, 0.0f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 1.0f, 0.0f } });
            filter->SetMeasurementNoise(math::SquareMatrix<float, 2>{
                { 0.1f, 0.0f },
                { 0.0f, 0.1f } });
            filter->SetProcessNoise(StateMat4{
                { 0.01f, 0.0f, 0.0f, 0.0f },
                { 0.0f, 0.01f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 0.01f, 0.0f },
                { 0.0f, 0.0f, 0.0f, 0.01f } });
        }
    };

    class KalmanFilterWithControlTest : public ::testing::Test
    {
    protected:
        using FilterCtrl = filters::KalmanFilter<float, 2, 1, 1>;
        using CtrlMat = math::Matrix<float, 2, 1>;
        using CtrlVec = math::Vector<float, 1>;

        StateMat2 A{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        MeasMat1x2 C{ { 1.0f, 0.0f } };
        CtrlMat B{
            { 0.0f },
            { 0.1f }
        };
        StateMat2 Q{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
        MeasCov1 R{ { 0.1f } };

        StateVec2 zeroState{};
        StateMat2 identity{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };

        std::optional<FilterCtrl> filter;

        void SetUp() override
        {
            filter.emplace(StateVec2{}, StateMat2{
                { 0.5f, 0.0f },
                { 0.0f, 0.5f } });
            filter->SetStateTransition(StateMat2{
                { 1.0f, kDt },
                { 0.0f, 1.0f } });
            filter->SetControlInputMatrix(CtrlMat{
                { 0.5f * kDt * kDt },
                { kDt } });
            filter->SetMeasurementMatrix(MeasMat1x2{ { 1.0f, 0.0f } });
            filter->SetMeasurementNoise(MeasCov1{ { 0.1f } });
            filter->SetProcessNoise(StateMat2{
                { 0.01f, 0.0f },
                { 0.0f, 0.01f } });
        }
    };

    class KalmanFilterConsistencyTest : public ::testing::Test
    {
    protected:
        using FilterType = filters::KalmanFilter<float, 2, 1>;
        using Cm1 = math::ConsistencyMetrics<float, 1>;
        using Cm2 = math::ConsistencyMetrics<float, 2>;
        using InnovVec = math::Vector<float, 1>;
        using InnovCov = math::SquareMatrix<float, 1>;

        StateMat2 F{
            { 1.0f, kDt },
            { 0.0f, 1.0f }
        };
        MeasMat1x2 H{ { 1.0f, 0.0f } };
        StateMat2 Q{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
        MeasCov1 R{ { 0.1f } };

        std::optional<FilterType> filter;

        void SetUp() override
        {
            filter.emplace(StateVec2{}, StateMat2{
                { 1.0f, 0.0f },
                { 0.0f, 1.0f } });
            filter->SetStateTransition(F);
            filter->SetMeasurementMatrix(H);
            filter->SetProcessNoise(Q);
            filter->SetMeasurementNoise(R);
        }
    };
}

TEST_F(KalmanFilterTest, DefaultInitializationPreservesStateAndCovariance)
{
    StateVec2 expectedState{};
    EXPECT_TRUE(AreVectorsNear(filter->GetState(), expectedState, kTolerance));
    EXPECT_TRUE(AreMatricesNear(filter->GetCovariance(), initialCovariance, kTolerance));
}

TEST_F(KalmanFilterTest, PredictAdvancesStateByConstantVelocity)
{
    filter.emplace(StateVec2{ { 0.0f }, { 0.1f } }, StateMat2{
        { 0.1f, 0.0f },
        { 0.0f, 0.1f } });
    filter->SetStateTransition(StateMat2{
        { 1.0f, kDt },
        { 0.0f, 1.0f } });

    filter->Predict();

    StateVec2 expected{ { 0.01f }, { 0.1f } };
    EXPECT_TRUE(AreVectorsNear(filter->GetState(), expected, kTolerance));
}

TEST_F(KalmanFilterTest, UpdateFusesInformationTowardMeasurement)
{
    filter->SetMeasurementMatrix(MeasMat1x2{ { 1.0f, 0.0f } });
    filter->SetMeasurementNoise(MeasCov1{ { 0.5f } });

    filter->Update(MeasVec1{ { 0.5f } });

    float posEstimate = filter->GetState().at(0, 0);
    EXPECT_GT(posEstimate, 0.0f);
    EXPECT_LT(posEstimate, 0.5f);
    EXPECT_LT(filter->GetCovariance().at(0, 0), 0.5f);
}

TEST_F(KalmanFilterTest, PredictUpdateCycleTracksMovingObject)
{
    ConfigureConstantVelocityModel();

    float truePos = 0.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * kDt;
        filter->Predict();
        filter->Update(MeasVec1{ { truePos } });
    }

    EXPECT_NEAR(filter->GetState().at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(filter->GetState().at(1, 0), trueVel, 0.4f);
}

TEST_F(KalmanFilterTest, PredictGrowsCovarianceThenUpdateReducesIt)
{
    ConfigureConstantVelocityModel();

    float p00Before = initialCovariance.at(0, 0);

    filter->Predict();
    float p00AfterPredict = filter->GetCovariance().at(0, 0);
    EXPECT_GT(p00AfterPredict, p00Before);

    filter->Update(MeasVec1{ { 0.1f } });
    EXPECT_LT(filter->GetCovariance().at(0, 0), p00AfterPredict);
}

TEST_F(KalmanFilterTest, RepeatedPredictGrowsAllDiagonalEntries)
{
    filter.emplace(StateVec2{}, StateMat2{
        { 0.1f, 0.0f },
        { 0.0f, 0.1f } });
    filter->SetStateTransition(StateMat2{
        { 1.0f, kDt },
        { 0.0f, 1.0f } });
    filter->SetProcessNoise(StateMat2{
        { 0.01f, 0.0f },
        { 0.0f, 0.01f } });

    for (int i = 0; i < 5; ++i)
        filter->Predict();

    EXPECT_GT(filter->GetCovariance().at(0, 0), 0.1f);
    EXPECT_GT(filter->GetCovariance().at(1, 1), 0.1f);
}

TEST_F(KalmanFilterTest, JosephFormKeepsCovariancePositiveSemiDefinite)
{
    filter.emplace(StateVec2{}, StateMat2{
        { 1000.0f, 0.0f },
        { 0.0f, 1000.0f } });
    filter->SetStateTransition(StateMat2{
        { 1.0f, kDt },
        { 0.0f, 1.0f } });
    filter->SetMeasurementMatrix(MeasMat1x2{ { 1.0f, 0.0f } });
    filter->SetMeasurementNoise(MeasCov1{ { 0.001f } });
    filter->SetProcessNoise(StateMat2{
        { 0.001f, 0.0f },
        { 0.0f, 0.001f } });

    for (int i = 0; i < 50; ++i)
    {
        filter->Predict();
        filter->Update(MeasVec1{ { 1.0f } });

        auto P = filter->GetCovariance();
        EXPECT_GE(P.at(0, 0), 0.0f);
        EXPECT_GE(P.at(1, 1), 0.0f);
        float det = P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0);
        EXPECT_GE(det, 0.0f);
    }
}

TEST_F(KalmanFilter3StateTest, ConstantAccelerationModelTracksPosition)
{
    float trueAcc = 1.0f;
    float trueVel = 0.0f;
    float truePos = 0.0f;

    for (int i = 0; i < 20; ++i)
    {
        trueVel += trueAcc * kDt;
        truePos += trueVel * kDt;
        filter->Predict();
        filter->Update(math::Vector<float, 1>{ { truePos } });
    }

    EXPECT_NEAR(filter->GetState().at(0, 0), truePos, 0.3f);
}

TEST_F(KalmanFilter4StateTest, TwoMeasurementModelTracksXYPosition)
{
    float trueX = 0.0f, trueVx = 0.5f;
    float trueY = 0.0f, trueVy = 0.3f;

    for (int i = 0; i < 20; ++i)
    {
        trueX += trueVx * kDt;
        trueY += trueVy * kDt;
        filter->Predict();
        filter->Update(math::Vector<float, 2>{ { trueX }, { trueY } });
    }

    EXPECT_NEAR(filter->GetState().at(0, 0), trueX, 0.2f);
    EXPECT_NEAR(filter->GetState().at(2, 0), trueY, 0.2f);
}

TEST_F(KalmanFilterWithControlTest, ControlInputAcceleratesStateEstimate)
{
    CtrlVec controlInput{ { 1.0f } };
    float truePos = 0.0f, trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * kDt;
        truePos += trueVel * kDt;
        filter->Predict(controlInput);
        filter->Update(MeasVec1{ { truePos } });
    }

    EXPECT_NEAR(filter->GetState().at(0, 0), truePos, 0.2f);
    EXPECT_NEAR(filter->GetState().at(1, 0), trueVel, 0.4f);
}

TEST_F(KalmanFilterWithControlTest, SetPlantEquivalentToIndividualSetters)
{
    filters::KalmanFilter<float, 2, 1, 1> filterA{ zeroState, identity };
    filters::KalmanFilter<float, 2, 1, 1> filterB{ zeroState, identity };

    filterA.SetStateTransition(A);
    filterA.SetMeasurementMatrix(C);
    filterA.SetControlInputMatrix(B);
    filterA.SetProcessNoise(Q);
    filterA.SetMeasurementNoise(R);

    auto plant = math::LinearTimeInvariant<float, 2, 1, 1>{ A, B, C, {} };
    filterB.SetPlant(plant);
    filterB.SetProcessNoise(Q);
    filterB.SetMeasurementNoise(R);

    math::Vector<float, 1> u{ { 1.0f } };
    filterA.Predict(u);
    filterB.Predict(u);

    EXPECT_NEAR(filterA.GetState().at(0, 0), filterB.GetState().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(filterA.GetState().at(1, 0), filterB.GetState().at(1, 0), math::Tolerance<float>());
}

TEST_F(KalmanFilterWithControlTest, SetPlantWithControlSizeZeroExtractsOnlyAandC)
{
    math::SquareMatrix<float, 2> F{
        { 0.9f, 0.1f },
        { 0.0f, 0.8f }
    };
    MeasMat1x2 H{ { 1.0f, 0.0f } };

    filters::KalmanFilter<float, 2, 1, 0> filterNoCtrl{ StateVec2{}, StateMat2{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f } } };

    math::LinearTimeInvariant<float, 2, 2, 1> plant;
    plant.A = F;
    plant.C = H;

    filterNoCtrl.SetPlant(plant);
    filterNoCtrl.SetProcessNoise(StateMat2{ { 0.01f, 0.0f }, { 0.0f, 0.01f } });
    filterNoCtrl.SetMeasurementNoise(MeasCov1{ { 0.1f } });

    filterNoCtrl.Predict();
    filterNoCtrl.Update(MeasVec1{ { 1.0f } });

    EXPECT_NE(filterNoCtrl.GetState().at(0, 0), 0.0f);
}

TEST_F(KalmanFilterConsistencyTest, NeesFallsInChiSquaredBandAfterConvergence)
{
    constexpr float truePos = 2.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 40; ++i)
    {
        filter->Predict();
        filter->Update(MeasVec1{ { truePos + trueVel * static_cast<float>(i) * kDt } });
    }

    StateVec2 truth{ { truePos + trueVel * 40.0f * kDt }, { trueVel } };
    StateVec2 error;
    error.at(0, 0) = filter->GetState().at(0, 0) - truth.at(0, 0);
    error.at(1, 0) = filter->GetState().at(1, 0) - truth.at(1, 0);

    auto nees = Cm2::Nees(error, filter->GetCovariance());
    ASSERT_TRUE(nees.has_value());
    EXPECT_GE(*nees, 0.0f);
}

TEST_F(KalmanFilterConsistencyTest, NisIsNonNegativeAndFiniteOnEveryStep)
{
    constexpr float trueVel = 1.0f;
    constexpr int kSteps = 30;

    for (int i = 0; i < kSteps; ++i)
    {
        filter->Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * kDt;
        MeasVec1 meas{ { measPos } };

        InnovVec innovation;
        innovation.at(0, 0) = measPos - filter->GetState().at(0, 0);

        auto P = filter->GetCovariance();
        InnovCov innovCov;
        innovCov.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, innovCov);
        ASSERT_TRUE(nis.has_value());
        EXPECT_GE(*nis, 0.0f);
        EXPECT_FALSE(std::isinf(*nis));
        EXPECT_FALSE(std::isnan(*nis));

        filter->Update(meas);
    }
}

TEST_F(KalmanFilterConsistencyTest, ZeroInnovationLeavesStateUnchangedAndReducesCovariance)
{
    filter->Predict();

    StateVec2 stateBefore = filter->GetState();
    float p00Before = filter->GetCovariance().at(0, 0);

    MeasVec1 exactMeas{ { filter->GetState().at(0, 0) } };
    filter->Update(exactMeas);

    EXPECT_NEAR(filter->GetState().at(0, 0), stateBefore.at(0, 0), math::Tolerance<float>());
    EXPECT_LT(filter->GetCovariance().at(0, 0), p00Before);
}

TEST_F(KalmanFilterConsistencyTest, CovarianceRemainsSymmetricAfterManySteps)
{
    constexpr float trueVel = 0.3f;

    for (int i = 0; i < 50; ++i)
    {
        filter->Predict();
        filter->Update(MeasVec1{ { trueVel * static_cast<float>(i) * kDt } });

        auto P = filter->GetCovariance();
        EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
    }
}

TEST_F(KalmanFilterConsistencyTest, SteadyStateCovarianceStabilizesAfterManySteps)
{
    constexpr float trueVel = 0.5f;
    for (int i = 0; i < 200; ++i)
    {
        filter->Predict();
        filter->Update(MeasVec1{ { trueVel * static_cast<float>(i) * kDt } });
    }

    auto P1 = filter->GetCovariance();

    for (int i = 0; i < 10; ++i)
    {
        filter->Predict();
        filter->Update(MeasVec1{ { trueVel * static_cast<float>(200 + i) * kDt } });
    }

    auto P2 = filter->GetCovariance();

    EXPECT_NEAR(P2.at(0, 0), P1.at(0, 0), 1e-4f);
    EXPECT_NEAR(P2.at(1, 1), P1.at(1, 1), 1e-4f);
    EXPECT_GE(P2.at(0, 0), 0.0f);
    EXPECT_GE(P2.at(1, 1), 0.0f);
}

TEST_F(KalmanFilterConsistencyTest, TwoIndependentInstancesProduceSameOutput)
{
    filters::KalmanFilter<float, 2, 1> filterA{ StateVec2{}, StateMat2{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f } } };
    filters::KalmanFilter<float, 2, 1> filterB{ StateVec2{}, StateMat2{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f } } };

    for (auto* f : { &filterA, &filterB })
    {
        f->SetStateTransition(F);
        f->SetMeasurementMatrix(H);
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

TEST_F(KalmanFilterConsistencyTest, NeesReturnNulloptForSingularCovariance)
{
    StateVec2 error{ { 1.0f }, { 0.0f } };
    StateMat2 singular{ { 0.0f, 0.0f }, { 0.0f, 0.0f } };

    auto nees = Cm2::Nees(error, singular);
    EXPECT_FALSE(nees.has_value());
}

TEST_F(KalmanFilterConsistencyTest, NisReturnNulloptForSingularInnovationCovariance)
{
    InnovVec innovation{ { 1.5f } };
    InnovCov singular{ { 0.0f } };

    auto nis = Cm1::Nis(innovation, singular);
    EXPECT_FALSE(nis.has_value());
}

TEST_F(KalmanFilterConsistencyTest, NeesIsNonNegativeForAnyValidErrorAndCovariance)
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

TEST_F(KalmanFilterConsistencyTest, TimeAveragedNisConsistentForNoiselessLinearSystem)
{
    constexpr int kSteps = 10;
    constexpr float trueVel = 0.2f;

    float nisSum = 0.0f;

    for (int i = 0; i < kSteps; ++i)
    {
        filter->Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * kDt;
        InnovVec innovation;
        innovation.at(0, 0) = measPos - filter->GetState().at(0, 0);

        auto P = filter->GetCovariance();
        InnovCov S;
        S.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, S);
        ASSERT_TRUE(nis.has_value());
        nisSum += *nis;

        filter->Update(MeasVec1{ { measPos } });
    }

    float avgNis = nisSum / static_cast<float>(kSteps);
    EXPECT_GE(avgNis, 0.0f);
}
