#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/nonlinear_control/ModelReferenceAdaptiveControl.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    static math::LinearTimeInvariant<float, 1, 1, 1> MakeFirstOrderReference()
    {
        math::LinearTimeInvariant<float, 1, 1, 1> lti{};
        lti.A.at(0, 0) = -1.0f;
        lti.B.at(0, 0) = 1.0f;
        return lti;
    }

    class TestModelReferenceAdaptiveControl : public ::testing::Test
    {
    protected:
        math::LinearTimeInvariant<float, 1, 1, 1> refModel{ MakeFirstOrderReference() };
        nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mrac{
            refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
        };
    };

    class TestModelReferenceAdaptiveControlMitRule : public ::testing::Test
    {
    protected:
        math::LinearTimeInvariant<float, 1, 1, 1> refModel{ MakeFirstOrderReference() };
        nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracMit{
            refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::MitRule
        };
    };
}

TEST_F(TestModelReferenceAdaptiveControl, reference_model_advances)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 0.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;
    const float dt{ 0.1f };

    mrac.ComputeControl(x, r, dt);

    const float expectedXm{ refModel.B.at(0, 0) * r.at(0, 0) * dt };
    EXPECT_NEAR(mrac.GetReferenceState().at(0, 0), expectedXm, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, zero_error_freezes_parameters)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 0.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.0f;

    mrac.ComputeControl(x, r, 0.1f);

    const float txBefore{ mrac.GetThetaX().at(0, 0) };
    const float trBefore{ mrac.GetThetaR().at(0, 0) };

    mrac.ComputeControl(x, r, 0.1f);

    EXPECT_NEAR(mrac.GetThetaX().at(0, 0), txBefore, math::Tolerance<float>());
    EXPECT_NEAR(mrac.GetThetaR().at(0, 0), trBefore, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, positive_error_adapts_feedback)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.0f;
    const float dt{ 0.1f };

    const float txBefore{ mrac.GetThetaX().at(0, 0) };

    mrac.ComputeControl(x, r, dt);

    const float xmAfter{ mrac.GetReferenceState().at(0, 0) };
    const float e{ x.at(0, 0) - xmAfter };
    const float expectedDelta{ -1.0f * 1.0f * e * x.at(0, 0) * dt };

    EXPECT_NEAR(mrac.GetThetaX().at(0, 0), txBefore + expectedDelta, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, feedforward_param_tracks_command)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;
    const float dt{ 0.1f };

    const float trBefore{ mrac.GetThetaR().at(0, 0) };

    mrac.ComputeControl(x, r, dt);

    const float xmAfter{ mrac.GetReferenceState().at(0, 0) };
    const float e{ x.at(0, 0) - xmAfter };
    const float expectedDelta{ -1.0f * 1.0f * e * r.at(0, 0) * dt };

    EXPECT_NEAR(mrac.GetThetaR().at(0, 0), trBefore + expectedDelta, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, signB_flips_adaptation_direction)
{
    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracNeg{
        refModel, 1.0f, -1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.0f;
    const float dt{ 0.1f };

    mrac.ComputeControl(x, r, dt);
    mracNeg.ComputeControl(x, r, dt);

    const float deltaPos{ mrac.GetThetaX().at(0, 0) };
    const float deltaNeg{ mracNeg.GetThetaX().at(0, 0) };

    EXPECT_NEAR(deltaPos, -deltaNeg, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, initial_control_output_is_zero_with_zero_thetas)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 3.5f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.2f;
    const float dt{ 0.05f };

    const auto u = mrac.ComputeControl(x, r, dt);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, control_output_matches_hand_computed_thetaX_thetaR)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 1.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.0f;
    const float dt{ 0.1f };
    const float gamma{ 1.0f };
    const float signB{ 1.0f };

    mrac.ComputeControl(x, r, dt);

    const float xmStep1{ 0.0f + (refModel.A.at(0, 0) * 0.0f + refModel.B.at(0, 0) * 0.0f) * dt };
    const float e1{ 1.0f - xmStep1 };
    const float thetaXAfter{ 0.0f - gamma * signB * dt * e1 * 1.0f };
    const float thetaRAfter{ 0.0f - gamma * signB * dt * e1 * 0.0f };

    math::Vector<float, 1> x2{};
    x2.at(0, 0) = 2.0f;
    math::Vector<float, 1> r2{};
    r2.at(0, 0) = 0.5f;

    const auto u2 = mrac.ComputeControl(x2, r2, dt);

    const float expectedU{ thetaXAfter * 2.0f + thetaRAfter * 0.5f };
    EXPECT_NEAR(u2.at(0, 0), expectedU, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, tracks_reference_over_time)
{
    float xPlant{ 2.0f };
    const float aPlant{ -1.5f };
    const float bPlant{ 2.0f };
    const float dt{ 0.01f };

    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    for (int k = 0; k < 2000; ++k)
    {
        math::Vector<float, 1> xVec{};
        xVec.at(0, 0) = xPlant;
        const auto u = mrac.ComputeControl(xVec, r, dt);
        xPlant += dt * (aPlant * xPlant + bPlant * u.at(0, 0));
    }

    const float xm{ mrac.GetReferenceState().at(0, 0) };
    EXPECT_LT(std::abs(xPlant - xm), 0.1f);
}

TEST_F(TestModelReferenceAdaptiveControl, reference_model_steady_state_equals_analytic_value)
{
    math::Vector<float, 1> x{};
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;
    const float dt{ 0.001f };

    for (int k = 0; k < 10000; ++k)
    {
        x.at(0, 0) = mrac.GetReferenceState().at(0, 0);
        mrac.ComputeControl(x, r, dt);
    }

    const float expectedSteadyState{ refModel.B.at(0, 0) / (-refModel.A.at(0, 0)) };
    EXPECT_NEAR(mrac.GetReferenceState().at(0, 0), expectedSteadyState, 1e-2f);
}

TEST_F(TestModelReferenceAdaptiveControl, gamma_scales_adaptation_speed)
{
    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracSlow{
        refModel, 0.5f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };
    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracFast{
        refModel, 5.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    float xSlow{ 2.0f };
    float xFast{ 2.0f };
    const float aPlant{ -1.5f };
    const float bPlant{ 2.0f };
    const float dt{ 0.005f };

    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    for (int k = 0; k < 400; ++k)
    {
        math::Vector<float, 1> xsVec{};
        xsVec.at(0, 0) = xSlow;
        const auto us = mracSlow.ComputeControl(xsVec, r, dt);
        xSlow += dt * (aPlant * xSlow + bPlant * us.at(0, 0));

        math::Vector<float, 1> xfVec{};
        xfVec.at(0, 0) = xFast;
        const auto uf = mracFast.ComputeControl(xfVec, r, dt);
        xFast += dt * (aPlant * xFast + bPlant * uf.at(0, 0));
    }

    const float errSlow{ std::abs(xSlow - mracSlow.GetReferenceState().at(0, 0)) };
    const float errFast{ std::abs(xFast - mracFast.GetReferenceState().at(0, 0)) };

    EXPECT_LT(errFast, errSlow);
}

TEST_F(TestModelReferenceAdaptiveControl, parameters_converge_under_excitation)
{
    const float aPlant{ -1.5f };
    const float bPlant{ 2.0f };
    const float amRef{ 1.0f };
    const float bmRef{ 1.0f };
    const float thetaXStar{ -(amRef - std::abs(aPlant)) / bPlant };
    const float thetaRStar{ bmRef / bPlant };

    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracPe{
        refModel, 2.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    float xPlant{ 0.0f };
    const float dt{ 0.005f };

    for (int k = 0; k < 4000; ++k)
    {
        const float t{ static_cast<float>(k) * dt };
        math::Vector<float, 1> r{};
        r.at(0, 0) = std::sin(t) + std::sin(3.0f * t);

        math::Vector<float, 1> xVec{};
        xVec.at(0, 0) = xPlant;
        const auto u = mracPe.ComputeControl(xVec, r, dt);
        xPlant += dt * (aPlant * xPlant + bPlant * u.at(0, 0));
    }

    EXPECT_NEAR(mracPe.GetThetaX().at(0, 0), thetaXStar, 0.5f);
    EXPECT_NEAR(mracPe.GetThetaR().at(0, 0), thetaRStar, 0.5f);
}

TEST_F(TestModelReferenceAdaptiveControl, reset_clears_state_and_params)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 3.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    mrac.ComputeControl(x, r, 0.1f);
    mrac.Reset();

    EXPECT_NEAR(mrac.GetReferenceState().at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(mrac.GetThetaX().at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(mrac.GetThetaR().at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControl, reset_restores_fresh_instance_behaviour)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;
    const float dt{ 0.1f };

    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> fresh{
        refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    mrac.ComputeControl(x, r, dt);
    mrac.Reset();
    const auto uAfterReset = mrac.ComputeControl(x, r, dt);
    const auto uFresh = fresh.ComputeControl(x, r, dt);

    EXPECT_FLOAT_EQ(uAfterReset.at(0, 0), uFresh.at(0, 0));
    EXPECT_FLOAT_EQ(
        mrac.GetReferenceState().at(0, 0), fresh.GetReferenceState().at(0, 0));
}

TEST_F(TestModelReferenceAdaptiveControl, determinism_same_input_same_output)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 1.5f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.8f;
    const float dt{ 0.05f };

    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mrac1{
        refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };
    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mrac2{
        refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    const auto u1 = mrac1.ComputeControl(x, r, dt);
    const auto u2 = mrac2.ComputeControl(x, r, dt);

    EXPECT_FLOAT_EQ(u1.at(0, 0), u2.at(0, 0));
    EXPECT_FLOAT_EQ(
        mrac1.GetReferenceState().at(0, 0), mrac2.GetReferenceState().at(0, 0));
}

TEST_F(TestModelReferenceAdaptiveControl, zero_dt_freezes_state_and_parameters)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    mrac.ComputeControl(x, r, 0.1f);

    const float xmBefore{ mrac.GetReferenceState().at(0, 0) };
    const float txBefore{ mrac.GetThetaX().at(0, 0) };
    const float trBefore{ mrac.GetThetaR().at(0, 0) };

    mrac.ComputeControl(x, r, 0.0f);

    EXPECT_FLOAT_EQ(mrac.GetReferenceState().at(0, 0), xmBefore);
    EXPECT_FLOAT_EQ(mrac.GetThetaX().at(0, 0), txBefore);
    EXPECT_FLOAT_EQ(mrac.GetThetaR().at(0, 0), trBefore);
}

TEST_F(TestModelReferenceAdaptiveControl, large_state_produces_finite_output)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 1.0e4f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0e4f;
    const float dt{ 0.01f };

    const auto u = mrac.ComputeControl(x, r, dt);

    EXPECT_TRUE(std::isfinite(u.at(0, 0)));
    EXPECT_TRUE(std::isfinite(mrac.GetReferenceState().at(0, 0)));
    EXPECT_TRUE(std::isfinite(mrac.GetThetaX().at(0, 0)));
    EXPECT_TRUE(std::isfinite(mrac.GetThetaR().at(0, 0)));
}

TEST_F(TestModelReferenceAdaptiveControlMitRule, mit_rule_reference_model_advances)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 0.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;
    const float dt{ 0.1f };

    mracMit.ComputeControl(x, r, dt);

    const float expectedXm{ refModel.B.at(0, 0) * r.at(0, 0) * dt };
    EXPECT_NEAR(mracMit.GetReferenceState().at(0, 0), expectedXm, math::Tolerance<float>());
}

TEST_F(TestModelReferenceAdaptiveControlMitRule, mit_rule_matches_lyapunov_update)
{
    nonlinear_control::ModelReferenceAdaptiveControl<float, 1, 1> mracLyap{
        refModel, 1.0f, +1.0f, nonlinear_control::AdaptationLaw::Lyapunov
    };

    math::Vector<float, 1> x{};
    x.at(0, 0) = 1.5f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 0.5f;
    const float dt{ 0.05f };

    const auto uMit = mracMit.ComputeControl(x, r, dt);
    const auto uLyap = mracLyap.ComputeControl(x, r, dt);

    EXPECT_FLOAT_EQ(uMit.at(0, 0), uLyap.at(0, 0));
    EXPECT_FLOAT_EQ(
        mracMit.GetThetaX().at(0, 0), mracLyap.GetThetaX().at(0, 0));
    EXPECT_FLOAT_EQ(
        mracMit.GetThetaR().at(0, 0), mracLyap.GetThetaR().at(0, 0));
}

TEST_F(TestModelReferenceAdaptiveControlMitRule, mit_rule_reset_clears_state)
{
    math::Vector<float, 1> x{};
    x.at(0, 0) = 2.0f;
    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    mracMit.ComputeControl(x, r, 0.1f);
    mracMit.Reset();

    EXPECT_NEAR(mracMit.GetReferenceState().at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(mracMit.GetThetaX().at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(mracMit.GetThetaR().at(0, 0), 0.0f, math::Tolerance<float>());
}
