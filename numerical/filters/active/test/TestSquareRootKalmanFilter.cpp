#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/filters/active/SquareRootKalmanFilter.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/Tolerance.hpp"
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
    using Cm2 = math::ConsistencyMetrics<float, 2>;
    using Cm1 = math::ConsistencyMetrics<float, 1>;
    using InnovVec = math::Vector<float, 1>;
    using InnovCov = math::SquareMatrix<float, 1>;

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

    class TestSquareRootKalmanFilterConsistency
        : public ::testing::Test
    {
    protected:
        StateMat S0 = math::CholeskyDecomposition<float, 2>::Factor(StateMat{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f } });
        StateMat Q{
            { 0.01f, 0.0f },
            { 0.0f, 0.01f }
        };
        MeasCov R{ { 0.1f } };

        SrkfType filter{ MakeX0(), S0 };

        void SetUp() override
        {
            filter.SetStateTransition(MakeF());
            filter.SetMeasurementMatrix(MakeH());
            filter.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(Q));
            filter.SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(R));
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

TEST_F(TestSquareRootKalmanFilter, identity_transition_predict_preserves_state)
{
    StateMat identity{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    StateVec x0{ { 3.0f }, { -1.5f } };
    SrkfType f2{ x0, S0 };
    f2.SetStateTransition(identity);
    f2.SetProcessNoiseFactor(StateMat{});

    f2.Predict();

    EXPECT_NEAR(f2.GetState().at(0, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(f2.GetState().at(1, 0), -1.5f, math::Tolerance<float>());
}

TEST_F(TestSquareRootKalmanFilter, zero_innovation_leaves_state_unchanged_and_reduces_covariance)
{
    SetupBothFilters();
    filter.Predict();

    StateVec stateBefore = filter.GetState();
    float p00Before = filter.GetCovariance().at(0, 0);

    MeasVec exactMeas{ { filter.GetState().at(0, 0) } };
    filter.Update(exactMeas);

    EXPECT_NEAR(filter.GetState().at(0, 0), stateBefore.at(0, 0), math::Tolerance<float>());
    EXPECT_LT(filter.GetCovariance().at(0, 0), p00Before);
}

TEST_F(TestSquareRootKalmanFilter, covariance_remains_symmetric_after_many_steps)
{
    SetupBothFilters();

    float pos = 0.0f;
    constexpr float vel = 0.3f;

    for (int step = 0; step < 50; ++step)
    {
        pos += vel * dt;
        filter.Predict();
        filter.Update(MeasVec{ { pos } });

        auto P = filter.GetCovariance();
        EXPECT_NEAR(P.at(0, 1), P.at(1, 0), math::Tolerance<float>());
    }
}

TEST_F(TestSquareRootKalmanFilter, two_independent_instances_produce_same_output)
{
    StateMat S0a = math::CholeskyDecomposition<float, 2>::Factor(MakeP0());
    StateMat S0b = math::CholeskyDecomposition<float, 2>::Factor(MakeP0());

    SrkfType filterA{ MakeX0(), S0a };
    SrkfType filterB{ MakeX0(), S0b };

    for (auto* f : { &filterA, &filterB })
    {
        f->SetStateTransition(MakeF());
        f->SetMeasurementMatrix(MakeH());
        f->SetProcessNoiseFactor(math::CholeskyDecomposition<float, 2>::Factor(MakeQ()));
        f->SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(MakeR()));
    }

    std::array<float, 5> measurements{ 0.1f, 0.25f, 0.42f, 0.61f, 0.83f };
    for (float z : measurements)
    {
        filterA.Predict();
        filterA.Update(MeasVec{ { z } });
        filterB.Predict();
        filterB.Update(MeasVec{ { z } });
    }

    EXPECT_FLOAT_EQ(filterA.GetState().at(0, 0), filterB.GetState().at(0, 0));
    EXPECT_FLOAT_EQ(filterA.GetState().at(1, 0), filterB.GetState().at(1, 0));
    EXPECT_FLOAT_EQ(filterA.GetCovariance().at(0, 0), filterB.GetCovariance().at(0, 0));
}

TEST_F(TestSquareRootKalmanFilter, long_horizon_no_nan_inf_drift)
{
    SetupBothFilters();

    float pos = 0.0f;
    constexpr float vel = 0.5f;

    for (int step = 0; step < 500; ++step)
    {
        pos += vel * dt;
        filter.Predict();
        filter.Update(MeasVec{ { pos } });
    }

    auto x = filter.GetState();
    auto P = filter.GetCovariance();

    EXPECT_TRUE(std::isfinite(x.at(0, 0)));
    EXPECT_TRUE(std::isfinite(x.at(1, 0)));
    EXPECT_TRUE(std::isfinite(P.at(0, 0)));
    EXPECT_TRUE(std::isfinite(P.at(1, 1)));
    EXPECT_GE(P.at(0, 0), 0.0f);
    EXPECT_GE(P.at(1, 1), 0.0f);
}

TEST_F(TestSquareRootKalmanFilter, estimate_converges_to_ground_truth)
{
    SetupBothFilters();

    float truePos = 0.0f;
    constexpr float trueVel = 0.5f;

    for (int step = 0; step < 50; ++step)
    {
        truePos += trueVel * dt;
        filter.Predict();
        filter.Update(MeasVec{ { truePos } });
    }

    auto x = filter.GetState();
    EXPECT_NEAR(x.at(0, 0), truePos, 0.15f);
    EXPECT_NEAR(x.at(1, 0), trueVel, 0.3f);
}

TEST_F(TestSquareRootKalmanFilterConsistency, nees_is_nonnegative_after_convergence)
{
    constexpr float truePos = 2.0f;
    constexpr float trueVel = 0.5f;

    for (int i = 0; i < 40; ++i)
    {
        filter.Predict();
        filter.Update(MeasVec{ { truePos + trueVel * static_cast<float>(i) * dt } });
    }

    StateVec truth{ { truePos + trueVel * 40.0f * dt }, { trueVel } };
    StateVec error;
    error.at(0, 0) = filter.GetState().at(0, 0) - truth.at(0, 0);
    error.at(1, 0) = filter.GetState().at(1, 0) - truth.at(1, 0);

    auto nees = Cm2::Nees(error, filter.GetCovariance());
    ASSERT_TRUE(nees.has_value());
    EXPECT_GE(*nees, 0.0f);
}

TEST_F(TestSquareRootKalmanFilterConsistency, nis_is_nonnegative_and_finite_on_every_step)
{
    constexpr float trueVel = 1.0f;
    constexpr int kSteps = 30;

    for (int i = 0; i < kSteps; ++i)
    {
        filter.Predict();

        float measPos = trueVel * static_cast<float>(i + 1) * dt;
        MeasVec meas{ { measPos } };

        InnovVec innovation;
        innovation.at(0, 0) = measPos - filter.GetState().at(0, 0);

        auto P = filter.GetCovariance();
        InnovCov innovCov;
        innovCov.at(0, 0) = P.at(0, 0) + R.at(0, 0);

        auto nis = Cm1::Nis(innovation, innovCov);
        ASSERT_TRUE(nis.has_value());
        EXPECT_GE(*nis, 0.0f);
        EXPECT_FALSE(std::isinf(*nis));
        EXPECT_FALSE(std::isnan(*nis));

        filter.Update(meas);
    }
}

TEST_F(TestSquareRootKalmanFilterConsistency, factor_is_lower_triangular_after_update)
{
    filter.Predict();
    filter.Update(MeasVec{ { 0.3f } });

    auto S = filter.GetCovarianceFactor();
    EXPECT_NEAR(S.at(0, 1), 0.0f, math::Tolerance<float>());
}

TEST_F(TestSquareRootKalmanFilterConsistency, factor_diagonal_is_positive)
{
    filter.Predict();
    filter.Update(MeasVec{ { 0.3f } });

    auto S = filter.GetCovarianceFactor();
    EXPECT_GT(S.at(0, 0), 0.0f);
    EXPECT_GT(S.at(1, 1), 0.0f);
}

TEST_F(TestSquareRootKalmanFilterConsistency, three_state_position_velocity_acceleration_tracking)
{
    using Srkf3 = filters::SquareRootKalmanFilter<float, 3, 1>;
    using StateVec3 = math::Vector<float, 3>;
    using StateMat3 = math::SquareMatrix<float, 3>;
    using MeasMat3 = math::Matrix<float, 1, 3>;

    StateMat3 P03{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    StateMat3 S03 = math::CholeskyDecomposition<float, 3>::Factor(P03);
    StateVec3 x03{};

    Srkf3 f3{ x03, S03 };
    f3.SetStateTransition(StateMat3{
        { 1.0f, dt, 0.5f * dt * dt },
        { 0.0f, 1.0f, dt },
        { 0.0f, 0.0f, 1.0f } });
    f3.SetMeasurementMatrix(MeasMat3{ { 1.0f, 0.0f, 0.0f } });
    f3.SetProcessNoiseFactor(math::CholeskyDecomposition<float, 3>::Factor(StateMat3{
        { 0.01f, 0.0f, 0.0f },
        { 0.0f, 0.01f, 0.0f },
        { 0.0f, 0.0f, 0.01f } }));
    f3.SetMeasurementNoiseFactor(math::CholeskyDecomposition<float, 1>::Factor(MeasCov{ { 0.1f } }));

    float trueAcc = 0.5f;
    float trueVel3 = 0.0f;
    float truePos3 = 0.0f;

    for (int i = 0; i < 30; ++i)
    {
        trueVel3 += trueAcc * dt;
        truePos3 += trueVel3 * dt;

        f3.Predict();
        f3.Update(MeasVec{ { truePos3 } });
    }

    auto finalState3 = f3.GetState();
    EXPECT_NEAR(finalState3.at(0, 0), truePos3, 0.15f);
    EXPECT_NEAR(finalState3.at(1, 0), trueVel3, 0.3f);
    EXPECT_NEAR(finalState3.at(2, 0), trueAcc, 0.5f);
}
