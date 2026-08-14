#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/DeadbeatControl.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestDeadbeatControl : public ::testing::Test
    {
    protected:
        static constexpr float kScalarA = 0.9f;
        static constexpr float kScalarB = 0.5f;

        static constexpr float kGainRefA = 100.0f;
        static constexpr float kGainRefB = -5.0f;
        static constexpr float kGainStateA = 100.0f;
        static constexpr float kGainStateB = 15.0f;

        math::SquareMatrix<float, 2> A2{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, 2, 1> B2{
            { 0.005f },
            { 0.1f }
        };
    };
}

TEST_F(TestDeadbeatControl, scalar_one_step_convergence)
{
    math::SquareMatrix<float, 1> A{ { kScalarA } };
    math::Matrix<float, 1, 1> B{ { kScalarB } };
    controllers::DeadbeatControl<float, 1, 1, 1> ctrl{ A, B };

    math::Vector<float, 1> x{ { 1.5f } };
    math::Vector<float, 1> r{ { 3.0f } };
    ctrl.SetReference(r);

    const auto u = ctrl.ComputeControl(x);
    const float xNext = kScalarA * x.at(0, 0) + kScalarB * u.at(0, 0);

    EXPECT_NEAR(xNext, r.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, two_state_two_step_convergence)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    math::Vector<float, 2> x{ { 0.0f }, { 0.0f } };
    math::Vector<float, 2> r{ { 5.0f }, { 0.0f } };
    ctrl.SetReference(r);

    for (std::size_t step = 0; step < 2; ++step)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A2 * x + B2 * u;
    }

    EXPECT_NEAR(x.at(0, 0), r.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), r.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, state_at_reference_produces_zero_control)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    math::Vector<float, 2> r{ { 5.0f }, { 0.0f } };
    ctrl.SetReference(r);

    const auto u = ctrl.ComputeControl(r);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, state_holds_at_reference_after_convergence)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    math::Vector<float, 2> x{ { 1.0f }, { 0.0f } };
    math::Vector<float, 2> r{ { 5.0f }, { 0.0f } };
    ctrl.SetReference(r);

    for (std::size_t step = 0; step < 2; ++step)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A2 * x + B2 * u;
    }

    for (std::size_t step = 0; step < 5; ++step)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A2 * x + B2 * u;
    }

    EXPECT_NEAR(x.at(0, 0), r.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), r.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, state_gain_matches_analytic_reachability_formula)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    EXPECT_NEAR(ctrl.GetStateGain().at(0, 0), kGainStateA, 1e-1f);
    EXPECT_NEAR(ctrl.GetStateGain().at(0, 1), kGainStateB, 1e-1f);
}

TEST_F(TestDeadbeatControl, reference_gain_matches_analytic_reachability_formula)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    EXPECT_NEAR(ctrl.GetReferenceGain().at(0, 0), kGainRefA, 1e-1f);
    EXPECT_NEAR(ctrl.GetReferenceGain().at(0, 1), kGainRefB, 1e-1f);
}

TEST_F(TestDeadbeatControl, lti_constructor_matches_matrix_constructor)
{
    auto plant = math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(A2, B2);

    controllers::DeadbeatControl<float, 2, 1, 2> ctrlMat{ A2, B2 };
    controllers::DeadbeatControl<float, 2, 1, 2> ctrlLti{ plant };

    EXPECT_NEAR(ctrlMat.GetStateGain().at(0, 0), ctrlLti.GetStateGain().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(ctrlMat.GetStateGain().at(0, 1), ctrlLti.GetStateGain().at(0, 1), math::Tolerance<float>());
    EXPECT_NEAR(ctrlMat.GetReferenceGain().at(0, 0), ctrlLti.GetReferenceGain().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(ctrlMat.GetReferenceGain().at(0, 1), ctrlLti.GetReferenceGain().at(0, 1), math::Tolerance<float>());
}
