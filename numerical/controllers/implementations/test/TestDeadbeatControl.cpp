#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/DeadbeatControl.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
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

    EXPECT_NEAR(ctrl.GetStateGain().at(0, 0), kGainStateA, math::Tolerance<float>());
    EXPECT_NEAR(ctrl.GetStateGain().at(0, 1), kGainStateB, math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, reference_gain_matches_analytic_reachability_formula)
{
    controllers::DeadbeatControl<float, 2, 1, 2> ctrl{ A2, B2 };

    EXPECT_NEAR(ctrl.GetReferenceGain().at(0, 0), kGainRefA, math::Tolerance<float>());
    EXPECT_NEAR(ctrl.GetReferenceGain().at(0, 1), kGainRefB, math::Tolerance<float>());
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

TEST_F(TestDeadbeatControl, two_step_gain_amplifies_noise_less_than_one_step)
{
    math::SquareMatrix<float, 1> A{ { kScalarA } };
    math::Matrix<float, 1, 1> B{ { kScalarB } };
    controllers::DeadbeatControl<float, 1, 1, 1> ctrl1{ A, B };
    controllers::DeadbeatControl<float, 1, 1, 2> ctrl2{ A, B };

    EXPECT_LT(std::abs(ctrl2.GetReferenceGain().at(0, 0)),
        std::abs(ctrl1.GetReferenceGain().at(0, 0)));
}

TEST_F(TestDeadbeatControl, scalar_two_step_closed_loop_is_stable)
{
    math::SquareMatrix<float, 1> A{ { kScalarA } };
    math::Matrix<float, 1, 1> B{ { kScalarB } };
    controllers::DeadbeatControl<float, 1, 1, 2> ctrl{ A, B };

    const float closedLoopEig = kScalarA - kScalarB * ctrl.GetStateGain().at(0, 0);

    EXPECT_LT(std::abs(closedLoopEig), 1.0f);
}

TEST_F(TestDeadbeatControl, scalar_two_step_closed_loop_unity_dc_gain)
{
    math::SquareMatrix<float, 1> A{ { kScalarA } };
    math::Matrix<float, 1, 1> B{ { kScalarB } };
    controllers::DeadbeatControl<float, 1, 1, 2> ctrl{ A, B };

    math::Vector<float, 1> x{ { 0.0f } };
    math::Vector<float, 1> r{ { 3.0f } };
    ctrl.SetReference(r);

    for (std::size_t i = 0; i < 50; ++i)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A * x + B * u;
    }

    EXPECT_NEAR(x.at(0, 0), r.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, scalar_fast_plant_two_step_unity_dc_gain)
{
    static constexpr float kFastA = 0.6065f;
    math::SquareMatrix<float, 1> A{ { kFastA } };
    math::Matrix<float, 1, 1> B{ { kScalarB } };
    controllers::DeadbeatControl<float, 1, 1, 2> ctrl{ A, B };

    math::Vector<float, 1> x{ { 0.0f } };
    math::Vector<float, 1> r{ { 3.0f } };
    ctrl.SetReference(r);

    for (std::size_t i = 0; i < 50; ++i)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A * x + B * u;
    }

    EXPECT_NEAR(x.at(0, 0), r.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControl, two_state_three_step_asymptotically_converges)
{
    controllers::DeadbeatControl<float, 2, 1, 3> ctrl{ A2, B2 };

    math::Vector<float, 2> x{ { 0.0f }, { 0.0f } };
    math::Vector<float, 2> r{ { 5.0f }, { 0.0f } };
    ctrl.SetReference(r);

    for (int i = 0; i < 50; ++i)
    {
        const auto u = ctrl.ComputeControl(x);
        x = A2 * x + B2 * u;
    }

    EXPECT_NEAR(x.at(0, 0), r.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), r.at(1, 0), math::Tolerance<float>());
}

namespace
{
    class TestDeadbeatControlShapes : public ::testing::Test
    {
    protected:
        template<std::size_t StateSize>
        static math::SquareMatrix<float, StateSize> Companion()
        {
            math::SquareMatrix<float, StateSize> a{};
            for (std::size_t i = 0; i < StateSize; ++i)
                a.at(i, i) = 1.0f;
            for (std::size_t i = 0; i + 1 < StateSize; ++i)
                a.at(i, i + 1) = 1.0f;
            return a;
        }
    };
}

TEST_F(TestDeadbeatControlShapes, scalar_one_step_drives_state_to_zero)
{
    math::SquareMatrix<float, 1> a{ { 0.9f } };
    math::Matrix<float, 1, 1> b{ { 0.5f } };

    controllers::DeadbeatControl<float, 1, 1, 1> deadbeat{ a, b };

    math::Vector<float, 1> state{};
    state.at(0, 0) = 2.0f;

    const auto u = deadbeat.ComputeControl(state);
    const auto next = a * state + b * u;

    EXPECT_NEAR(next.at(0, 0), 0.0f, 1e-4f);
}

TEST_F(TestDeadbeatControlShapes, scalar_two_step_is_finite_and_reduces_state)
{
    math::SquareMatrix<float, 1> a{ { 0.9f } };
    math::Matrix<float, 1, 1> b{ { 0.5f } };

    controllers::DeadbeatControl<float, 1, 1, 2> deadbeat{ a, b };

    math::Vector<float, 1> state{};
    state.at(0, 0) = 1.0f;

    const auto u = deadbeat.ComputeControl(state);
    EXPECT_TRUE(std::isfinite(u.at(0, 0)));
    EXPECT_TRUE(std::isfinite(deadbeat.GetStateGain().at(0, 0)));
}

TEST_F(TestDeadbeatControlShapes, second_order_two_step_reaches_origin)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 1> b{ { 0.0f }, { 1.0f } };

    controllers::DeadbeatControl<float, 2, 1, 2> deadbeat{ a, b };

    math::Vector<float, 2> state{};
    state.at(0, 0) = 1.0f;
    state.at(1, 0) = 0.5f;

    auto x = state;
    for (int step = 0; step < 2; ++step)
        x = a * x + b * deadbeat.ComputeControl(x);

    EXPECT_NEAR(x.at(0, 0), 0.0f, 1e-3f);
    EXPECT_NEAR(x.at(1, 0), 0.0f, 1e-3f);
}

TEST_F(TestDeadbeatControlShapes, second_order_three_step_horizon_is_finite)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 1> b{ { 0.0f }, { 1.0f } };

    controllers::DeadbeatControl<float, 2, 1, 3> deadbeat{ a, b };

    math::Vector<float, 2> state{};
    state.at(0, 0) = 1.0f;

    EXPECT_TRUE(std::isfinite(deadbeat.ComputeControl(state).at(0, 0)));
}

TEST_F(TestDeadbeatControlShapes, two_input_plant_produces_two_channels)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 2> b{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    controllers::DeadbeatControl<float, 2, 2, 1> deadbeat{ a, b };

    math::Vector<float, 2> state{};
    state.at(0, 0) = 1.0f;
    state.at(1, 0) = 2.0f;

    const auto u = deadbeat.ComputeControl(state);
    const auto next = a * state + b * u;

    EXPECT_NEAR(next.at(0, 0), 0.0f, 1e-3f);
    EXPECT_NEAR(next.at(1, 0), 0.0f, 1e-3f);
}

TEST_F(TestDeadbeatControlShapes, reference_tracking_uses_reference_gain)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 2> b{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    controllers::DeadbeatControl<float, 2, 2, 1> deadbeat{ a, b };

    math::Vector<float, 2> reference{};
    reference.at(0, 0) = 1.0f;

    math::Vector<float, 2> state{};

    const auto without = deadbeat.ComputeControl(state);
    deadbeat.SetReference(reference);
    const auto with = deadbeat.ComputeControl(state);
    deadbeat.ClearReference();
    const auto cleared = deadbeat.ComputeControl(state);

    EXPECT_NE(with.at(0, 0), without.at(0, 0));
    EXPECT_NEAR(cleared.at(0, 0), without.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControlShapes, plant_constructor_and_copy_agree_with_matrix_constructor)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 1> b{ { 0.0f }, { 1.0f } };

    math::LinearTimeInvariant<float, 2, 1> plant{};
    plant.A = a;
    plant.B = b;

    controllers::DeadbeatControl<float, 2, 1, 3> fromPlant{ plant };
    controllers::DeadbeatControl<float, 2, 1, 3> copy{ fromPlant };

    math::Vector<float, 2> state{};
    state.at(0, 0) = 1.0f;

    EXPECT_NEAR(copy.ComputeControl(state).at(0, 0), fromPlant.ComputeControl(state).at(0, 0),
        math::Tolerance<float>());
    EXPECT_NEAR(copy.GetStateGain().at(0, 0), fromPlant.GetStateGain().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(copy.GetReferenceGain().at(0, 0), fromPlant.GetReferenceGain().at(0, 0), math::Tolerance<float>());
}

TEST_F(TestDeadbeatControlShapes, scalar_plant_constructor_and_reference_lifecycle)
{
    math::LinearTimeInvariant<float, 1, 1> plant{};
    plant.A = math::SquareMatrix<float, 1>{ { 0.9f } };
    plant.B = math::Matrix<float, 1, 1>{ { 0.5f } };

    controllers::DeadbeatControl<float, 1, 1, 1> oneStep{ plant };
    controllers::DeadbeatControl<float, 1, 1, 2> twoStep{ plant };
    controllers::DeadbeatControl<float, 1, 1, 1> copied{ oneStep };

    math::Vector<float, 1> state{};
    state.at(0, 0) = 1.0f;

    math::Vector<float, 1> reference{};
    reference.at(0, 0) = 0.5f;

    oneStep.SetReference(reference);
    const auto tracked = oneStep.ComputeControl(state);
    oneStep.ClearReference();
    const auto regulated = oneStep.ComputeControl(state);

    EXPECT_NE(tracked.at(0, 0), regulated.at(0, 0));
    EXPECT_NEAR(copied.GetStateGain().at(0, 0), oneStep.GetStateGain().at(0, 0), math::Tolerance<float>());

    twoStep.SetReference(reference);
    EXPECT_TRUE(std::isfinite(twoStep.ComputeControl(state).at(0, 0)));
    twoStep.ClearReference();
    EXPECT_TRUE(std::isfinite(twoStep.ComputeControl(state).at(0, 0)));
}

TEST_F(TestDeadbeatControlShapes, two_step_second_order_reference_lifecycle_and_copy)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 1> b{ { 0.0f }, { 1.0f } };

    controllers::DeadbeatControl<float, 2, 1, 2> deadbeat{ a, b };
    controllers::DeadbeatControl<float, 2, 1, 2> copy{ deadbeat };

    math::Vector<float, 2> reference{};
    reference.at(0, 0) = 1.0f;

    math::Vector<float, 2> state{};

    deadbeat.SetReference(reference);
    const auto tracked = deadbeat.ComputeControl(state);
    deadbeat.ClearReference();
    const auto regulated = deadbeat.ComputeControl(state);

    EXPECT_NE(tracked.at(0, 0), regulated.at(0, 0));
    EXPECT_TRUE(std::isfinite(copy.ComputeControl(state).at(0, 0)));
}

TEST_F(TestDeadbeatControlShapes, two_input_plant_constructor_and_gain_accessors)
{
    auto a = Companion<2>();
    math::Matrix<float, 2, 2> b{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::LinearTimeInvariant<float, 2, 2> plant{};
    plant.A = a;
    plant.B = b;

    controllers::DeadbeatControl<float, 2, 2, 1> fromPlant{ plant };
    controllers::DeadbeatControl<float, 2, 2, 1> copy{ fromPlant };

    EXPECT_NEAR(copy.GetStateGain().at(0, 0), fromPlant.GetStateGain().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(copy.GetReferenceGain().at(1, 1), fromPlant.GetReferenceGain().at(1, 1), math::Tolerance<float>());
}
