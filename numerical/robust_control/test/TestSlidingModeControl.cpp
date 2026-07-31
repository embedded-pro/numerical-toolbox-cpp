#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/robust_control/SlidingModeControl.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    static constexpr float kDt{ 0.01f };
    static constexpr float kPhi{ 0.05f };
    static constexpr float kK{ 0.04f };

    math::LinearTimeInvariant<float, 2, 1, 2> MakeDoubleIntegrator()
    {
        math::LinearTimeInvariant<float, 2, 1, 2> plant{};
        plant.A.at(0, 0) = 1.0f;
        plant.A.at(0, 1) = kDt;
        plant.A.at(1, 1) = 1.0f;
        plant.B.at(0, 0) = 0.5f * kDt * kDt;
        plant.B.at(1, 0) = kDt;
        plant.C = math::Matrix<float, 2, 2>::Identity();
        return plant;
    }

    class TestSlidingModeControl : public ::testing::Test
    {
    protected:
        math::LinearTimeInvariant<float, 2, 1, 2> plant{ MakeDoubleIntegrator() };
        math::Matrix<float, 1, 2> S{ { 1.0f, 1.0f } };
        math::Vector<float, 1> K{ { kK } };
        robust_control::SlidingModeControl<float, 2, 1> smc{ plant, S, K, kPhi };
    };
}

TEST_F(TestSlidingModeControl, reaches_sliding_surface)
{
    math::Vector<float, 2> x{ { 1.0f }, { 0.0f } };
    bool entered{ false };

    for (int i = 0; i < 500; ++i)
    {
        auto u = smc.ComputeControl(x);
        x = plant.Step(x, u);

        auto sv = smc.Surface(x);
        if (std::abs(sv.at(0, 0)) <= kPhi)
        {
            entered = true;
            break;
        }
    }

    EXPECT_TRUE(entered);
}

TEST_F(TestSlidingModeControl, stays_in_boundary_layer)
{
    math::Vector<float, 2> x{ { 0.02f }, { -0.02f } };

    for (int i = 0; i < 100; ++i)
    {
        auto sv = smc.Surface(x);
        EXPECT_LE(std::abs(sv.at(0, 0)), kPhi + math::Tolerance<float>());

        auto u = smc.ComputeControl(x);
        x = plant.Step(x, u);
    }
}

TEST_F(TestSlidingModeControl, equivalent_control_holds_surface)
{
    math::Vector<float, 2> x{ { 1.0f }, { -1.0f } };

    auto sv = smc.Surface(x);
    EXPECT_NEAR(sv.at(0, 0), 0.0f, math::Tolerance<float>());

    auto u = smc.ComputeControl(x);
    auto xNext = plant.Step(x, u);
    auto svNext = smc.Surface(xNext);

    EXPECT_NEAR(svNext.at(0, 0), 0.0f, 1e-4f);
}

TEST_F(TestSlidingModeControl, rejects_matched_disturbance)
{
    math::Vector<float, 2> x{ { 1.0f }, { 0.0f } };
    const float disturbance{ 0.01f };
    bool reached{ false };

    for (int i = 0; i < 500; ++i)
    {
        auto u = smc.ComputeControl(x);
        math::Vector<float, 1> uPerturbed{ { u.at(0, 0) + disturbance } };
        x = plant.Step(x, uPerturbed);

        auto sv = smc.Surface(x);
        if (std::abs(sv.at(0, 0)) <= kPhi)
        {
            reached = true;
            break;
        }
    }

    EXPECT_TRUE(reached);

    const float xNorm = std::sqrt(x.at(0, 0) * x.at(0, 0) + x.at(1, 0) * x.at(1, 0));
    EXPECT_LT(xNorm, 10.0f);
}

TEST_F(TestSlidingModeControl, boundary_layer_suppresses_chattering)
{
    math::Vector<float, 2> x{ { 0.01f }, { -0.01f } };

    auto u0 = smc.ComputeControl(x);
    x = plant.Step(x, u0);
    auto u1 = smc.ComputeControl(x);

    EXPECT_NEAR(u0.at(0, 0), u1.at(0, 0), 0.5f);
    EXPECT_GT(u0.at(0, 0) * u1.at(0, 0), -0.1f);
}

TEST_F(TestSlidingModeControl, larger_phi_increases_boundary_error)
{
    math::Vector<float, 2> x0{ { 0.5f }, { 0.0f } };

    const float phi1{ 0.05f };
    const float phi2{ 0.20f };
    robust_control::SlidingModeControl<float, 2, 1> smc1{ plant, S, K, phi1 };
    robust_control::SlidingModeControl<float, 2, 1> smc2{ plant, S, K, phi2 };

    math::Vector<float, 2> x1{ x0 };
    math::Vector<float, 2> x2{ x0 };

    for (int i = 0; i < 300; ++i)
    {
        x1 = plant.Step(x1, smc1.ComputeControl(x1));
        x2 = plant.Step(x2, smc2.ComputeControl(x2));
    }

    const float s1 = std::abs(smc1.Surface(x1).at(0, 0));
    const float s2 = std::abs(smc2.Surface(x2).at(0, 0));

    EXPECT_LE(s1, s2 + math::Tolerance<float>());
}

TEST_F(TestSlidingModeControl, surface_gain_sets_sliding_dynamics)
{
    math::Vector<float, 2> x{ { 1.0f }, { -1.0f } };

    auto u = smc.ComputeControl(x);

    const float sDot = (S * (plant.A * x + plant.B * u)).at(0, 0);
    EXPECT_NEAR(sDot, 0.0f, 1e-4f);
}

TEST_F(TestSlidingModeControl, control_sign_opposes_surface)
{
    math::Vector<float, 2> x{ { 0.5f }, { 0.5f } };

    auto sv = smc.Surface(x);
    EXPECT_GT(sv.at(0, 0), kPhi);

    auto u = smc.ComputeControl(x);
    const float sDot = (S * (plant.A * x + plant.B * u)).at(0, 0);

    EXPECT_LT(sDot, 0.0f);
}

TEST_F(TestSlidingModeControl, reference_tracking_acts_on_error)
{
    math::Vector<float, 2> x{ { 1.2f }, { 0.3f } };
    math::Vector<float, 2> reference{ { 0.4f }, { -0.1f } };
    math::Vector<float, 2> error{ { x.at(0, 0) - reference.at(0, 0) }, { x.at(1, 0) - reference.at(1, 0) } };

    auto uRef = smc.ComputeControl(x, reference);
    auto uErr = smc.ComputeControl(error);

    EXPECT_NEAR(uRef.at(0, 0), uErr.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestSlidingModeControl, set_boundary_layer_matches_constructed_phi)
{
    math::Vector<float, 2> x{ { 0.02f }, { -0.015f } };
    const float phi2{ 0.20f };

    robust_control::SlidingModeControl<float, 2, 1> reference{ plant, S, K, phi2 };
    smc.SetBoundaryLayer(phi2);

    auto uSet = smc.ComputeControl(x);
    auto uRef = reference.ComputeControl(x);

    EXPECT_NEAR(uSet.at(0, 0), uRef.at(0, 0), math::Tolerance<float>());
}
