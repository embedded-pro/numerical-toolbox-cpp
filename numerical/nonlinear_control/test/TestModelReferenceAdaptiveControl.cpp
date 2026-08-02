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

TEST_F(TestModelReferenceAdaptiveControl, control_law_combines_terms)
{
    float xPlant{ 0.5f };
    const float aPlant{ -1.5f };
    const float bPlant{ 2.0f };
    const float dt{ 0.01f };

    math::Vector<float, 1> r{};
    r.at(0, 0) = 1.0f;

    for (int k = 0; k < 500; ++k)
    {
        math::Vector<float, 1> xVec{};
        xVec.at(0, 0) = xPlant;
        const auto u = mrac.ComputeControl(xVec, r, dt);
        xPlant += dt * (aPlant * xPlant + bPlant * u.at(0, 0));
    }

    math::Vector<float, 1> xNow{};
    xNow.at(0, 0) = xPlant;
    const float txCurrent{ mrac.GetThetaX().at(0, 0) };
    const float trCurrent{ mrac.GetThetaR().at(0, 0) };
    const auto u = mrac.ComputeControl(xNow, r, dt);

    const float expectedU{ txCurrent * xPlant + trCurrent * r.at(0, 0) };
    EXPECT_NEAR(u.at(0, 0), expectedU, math::Tolerance<float>());
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
