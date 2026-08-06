#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/filters/active/SquareRootKalmanFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    constexpr float dt = 0.1f;

    using SrkfType = filters::SquareRootKalmanFilter<float, 2, 1>;
    using KfType = filters::KalmanFilter<float, 2, 1>;
    using StateVec = math::Vector<float, 2>;
    using StateMat = math::SquareMatrix<float, 2>;
    using MeasVec = math::Vector<float, 1>;
    using MeasMat = math::Matrix<float, 1, 2>;
    using MeasCov = math::SquareMatrix<float, 1>;

    StateMat MakeF()
    {
        return StateMat{
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
    }

    MeasMat MakeH()
    {
        return MeasMat{ { 1.0f, 0.0f } };
    }

    StateMat MakeQ()
    {
        return StateMat{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
    }

    MeasCov MakeR()
    {
        return MeasCov{ { 0.5f } };
    }

    StateVec MakeX0()
    {
        return StateVec{ { 0.0f }, { 0.0f } };
    }

    StateMat MakeP0()
    {
        return StateMat{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };
    }

    float MatrixTrace(const StateMat& m)
    {
        return m.at(0, 0) + m.at(1, 1);
    }

    bool IsPositiveDefinite(const StateMat& m)
    {
        float d00 = m.at(0, 0);
        float det = m.at(0, 0) * m.at(1, 1) - m.at(0, 1) * m.at(1, 0);
        return d00 > 0.0f && det > 0.0f;
    }

    class TestSquareRootKalmanFilter
        : public ::testing::Test
    {
    protected:
        StateMat S0 = math::CholeskyDecomposition<float, 2>::Factor(MakeP0());

        SrkfType filter{ MakeX0(), S0 };
        KfType reference{ MakeX0(), MakeP0() };

        void SetupBothFilters()
        {
            filter.SetStateTransition(MakeF());
            filter.SetMeasurementMatrix(MakeH());
            filter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
            filter.SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(MakeR()));

            reference.SetStateTransition(MakeF());
            reference.SetMeasurementMatrix(MakeH());
            reference.SetProcessNoise(MakeQ());
            reference.SetMeasurementNoise(MakeR());
        }
    };
}

TEST_F(TestSquareRootKalmanFilter, matches_conventional_kalman)
{
    SetupBothFilters();

    float pos = 0.0f;
    float vel = 0.5f;

    for (int step = 0; step < 10; ++step)
    {
        pos += vel * dt;
        MeasVec z{ { pos } };

        filter.Predict();
        filter.Update(z);

        reference.Predict();
        reference.Update(z);
    }

    auto sx = filter.GetState();
    auto rx = reference.GetState();
    EXPECT_NEAR(sx.at(0, 0), rx.at(0, 0), 5e-3f);
    EXPECT_NEAR(sx.at(1, 0), rx.at(1, 0), 5e-3f);

    auto sP = filter.GetCovariance();
    auto rP = reference.GetCovariance();
    EXPECT_NEAR(sP.at(0, 0), rP.at(0, 0), 5e-3f);
    EXPECT_NEAR(sP.at(1, 1), rP.at(1, 1), 5e-3f);
}

TEST_F(TestSquareRootKalmanFilter, covariance_stays_positive_definite)
{
    SetupBothFilters();

    float pos = 0.0f;
    float vel = 0.3f;

    for (int step = 0; step < 30; ++step)
    {
        pos += vel * dt;
        MeasVec z{ { pos } };

        filter.Predict();
        filter.Update(z);

        auto P = filter.GetCovariance();
        EXPECT_TRUE(IsPositiveDefinite(P));
    }
}

TEST_F(TestSquareRootKalmanFilter, factor_reconstructs_covariance)
{
    SetupBothFilters();

    MeasVec z{ { 1.0f } };
    filter.Predict();
    filter.Update(z);
    filter.Predict();
    filter.Update(z);

    auto S = filter.GetCovarianceFactor();
    auto P = filter.GetCovariance();
    auto SSt = S * S.Transpose();

    EXPECT_NEAR(SSt.at(0, 0), P.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(SSt.at(0, 1), P.at(0, 1), math::Tolerance<float>());
    EXPECT_NEAR(SSt.at(1, 0), P.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(SSt.at(1, 1), P.at(1, 1), math::Tolerance<float>());
}

TEST_F(TestSquareRootKalmanFilter, predict_grows_uncertainty)
{
    filter.SetStateTransition(MakeF());
    filter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));

    float traceBefore = MatrixTrace(filter.GetCovariance());
    filter.Predict();
    float traceAfter = MatrixTrace(filter.GetCovariance());

    EXPECT_GT(traceAfter, traceBefore);
}

TEST_F(TestSquareRootKalmanFilter, update_shrinks_uncertainty)
{
    SetupBothFilters();

    filter.Predict();
    float traceAfterPredict = MatrixTrace(filter.GetCovariance());

    MeasVec z{ { 0.05f } };
    filter.Update(z);
    float traceAfterUpdate = MatrixTrace(filter.GetCovariance());

    EXPECT_LT(traceAfterUpdate, traceAfterPredict);
}

TEST_F(TestSquareRootKalmanFilter, ill_conditioned_stays_stable)
{
    StateMat P0ill{
        { 1e8f, 0.0f },
        { 0.0f, 1.0f }
    };
    StateMat S0ill = math::CholeskyDecomposition<float, 2>::Factor(P0ill);

    SrkfType illFilter{ MakeX0(), S0ill };
    illFilter.SetStateTransition(MakeF());
    illFilter.SetMeasurementMatrix(MakeH());
    illFilter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
    illFilter.SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(MakeR()));

    for (int step = 0; step < 20; ++step)
    {
        MeasVec z{ { float(step) * 0.05f } };
        illFilter.Predict();
        illFilter.Update(z);
    }

    auto x = illFilter.GetState();
    auto P = illFilter.GetCovariance();

    EXPECT_TRUE(std::isfinite(x.at(0, 0)));
    EXPECT_TRUE(std::isfinite(x.at(1, 0)));
    EXPECT_TRUE(IsPositiveDefinite(P));
}

TEST_F(TestSquareRootKalmanFilter, perfect_measurement_collapses_variance)
{
    MeasCov sqrtRtiny{ { 1e-5f } };

    filter.SetStateTransition(MakeF());
    filter.SetMeasurementMatrix(MakeH());
    filter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
    filter.SetMeasurementNoiseFactor(sqrtRtiny);

    filter.Predict();
    MeasVec z{ { 0.5f } };
    filter.Update(z);

    auto P = filter.GetCovariance();
    EXPECT_LT(P.at(0, 0), 1e-4f);
    EXPECT_TRUE(std::isfinite(filter.GetState().at(0, 0)));
}

TEST_F(TestSquareRootKalmanFilter, vague_measurement_is_ignored)
{
    MeasCov sqrtRlarge{ { 1e4f } };

    filter.SetStateTransition(MakeF());
    filter.SetMeasurementMatrix(MakeH());
    filter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
    filter.SetMeasurementNoiseFactor(sqrtRlarge);

    filter.Predict();
    auto xPred = filter.GetState();

    MeasVec z{ { 999.0f } };
    filter.Update(z);
    auto xPost = filter.GetState();

    EXPECT_NEAR(xPost.at(0, 0), xPred.at(0, 0), 0.1f);
    EXPECT_NEAR(xPost.at(1, 0), xPred.at(1, 0), 0.1f);
}

TEST_F(TestSquareRootKalmanFilter, steady_state_gain_converges)
{
    SetupBothFilters();

    float tracePrev = 1e30f;
    float traceConverged = 0.0f;
    int convergedSteps = 0;

    for (int step = 0; step < 100; ++step)
    {
        MeasVec z{ { 1.0f } };
        filter.Predict();
        filter.Update(z);

        float traceNow = MatrixTrace(filter.GetCovariance());
        float diff = std::abs(traceNow - tracePrev);
        if (diff < 1e-6f)
            ++convergedSteps;
        else
            convergedSteps = 0;

        tracePrev = traceNow;
        if (convergedSteps >= 5)
        {
            traceConverged = traceNow;
            break;
        }
    }

    EXPECT_GT(convergedSteps, 0);
    EXPECT_GT(traceConverged, 0.0f);
}

TEST_F(TestSquareRootKalmanFilter, control_input_matches_conventional_kalman)
{
    using SrkfCtrl = filters::SquareRootKalmanFilter<float, 2, 1, 1>;
    using KfCtrl = filters::KalmanFilter<float, 2, 1, 1>;
    using CtrlMat = math::Matrix<float, 2, 1>;
    using CtrlVec = math::Vector<float, 1>;

    CtrlMat B{ { 0.5f * dt * dt }, { dt } };

    SrkfCtrl srkf{ MakeX0(), S0 };
    srkf.SetStateTransition(MakeF());
    srkf.SetMeasurementMatrix(MakeH());
    srkf.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
    srkf.SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(MakeR()));
    srkf.SetControlInputMatrix(B);

    KfCtrl reference2{ MakeX0(), MakeP0() };
    reference2.SetStateTransition(MakeF());
    reference2.SetMeasurementMatrix(MakeH());
    reference2.SetProcessNoise(MakeQ());
    reference2.SetMeasurementNoise(MakeR());
    reference2.SetControlInputMatrix(B);

    CtrlVec u{ { 1.0f } };
    float pos = 0.0f;
    float vel = 0.0f;

    for (int step = 0; step < 10; ++step)
    {
        vel += dt;
        pos += vel * dt;
        MeasVec z{ { pos } };

        srkf.Predict(u);
        srkf.Update(z);

        reference2.Predict(u);
        reference2.Update(z);
    }

    auto sx = srkf.GetState();
    auto rx = reference2.GetState();
    EXPECT_NEAR(sx.at(0, 0), rx.at(0, 0), 5e-3f);
    EXPECT_NEAR(sx.at(1, 0), rx.at(1, 0), 5e-3f);
}

TEST_F(TestSquareRootKalmanFilter, reset_and_getters)
{
    auto x = filter.GetState();
    auto S = filter.GetCovarianceFactor();

    EXPECT_NEAR(x.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 0.0f, math::Tolerance<float>());

    EXPECT_NEAR(S.at(0, 0), S0.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(S.at(1, 0), S0.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(S.at(0, 1), S0.at(0, 1), math::Tolerance<float>());
    EXPECT_NEAR(S.at(1, 1), S0.at(1, 1), math::Tolerance<float>());
}
