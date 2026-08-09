#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include "numerical/controllers/interfaces/StateFeedbackController.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>
#include <type_traits>

namespace
{
    class TestStateFeedbackController : public ::testing::Test
    {
    protected:
        math::SquareMatrix<float, 2> A{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, 2, 1> B{
            { 0.0f },
            { 0.1f }
        };
        math::SquareMatrix<float, 2> Q{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };
        math::SquareMatrix<float, 1> R{ { 0.1f } };
    };
}

TEST_F(TestStateFeedbackController, lqr_is_derived_from_state_feedback_controller)
{
    static_assert(
        std::is_base_of_v<
            controllers::StateFeedbackController<float, 2, 1>,
            controllers::Lqr<float, 2, 1>>,
        "Lqr must derive from StateFeedbackController");
}

TEST_F(TestStateFeedbackController, mpc_is_derived_from_state_feedback_controller)
{
    static_assert(
        std::is_base_of_v<
            controllers::StateFeedbackController<float, 2, 1>,
            controllers::Mpc<float, 2, 1, 5, 5>>,
        "Mpc must derive from StateFeedbackController");
}

TEST_F(TestStateFeedbackController, interface_is_abstract)
{
    static_assert(
        std::is_abstract_v<controllers::StateFeedbackController<float, 2, 1>>,
        "StateFeedbackController must be abstract");
}

TEST_F(TestStateFeedbackController, interface_destructor_is_virtual_not_pure)
{
    static_assert(
        !std::is_abstract_v<controllers::Lqr<float, 2, 1>>,
        "Concrete Lqr must be instantiable through base pointer");
}

TEST_F(TestStateFeedbackController, lqr_compute_control_returns_nonzero_for_nonzero_state)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NE(u.at(0, 0), 0.0f);
}

TEST_F(TestStateFeedbackController, lqr_zero_state_produces_zero_control)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 0.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestStateFeedbackController, lqr_positive_position_state_yields_negative_control)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_LT(u.at(0, 0), 0.0f);
}

TEST_F(TestStateFeedbackController, lqr_negative_position_state_yields_positive_control)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { -1.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_GT(u.at(0, 0), 0.0f);
}

TEST_F(TestStateFeedbackController, lqr_gain_matches_dare_reference)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    const auto& K = lqr.GetGain();

    EXPECT_NEAR(K.at(0, 0), 2.5853f, 1e-2f);
    EXPECT_NEAR(K.at(0, 1), 3.5747f, 1e-2f);
}

TEST_F(TestStateFeedbackController, lqr_control_matches_dare_reference_for_position_state)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -2.5853f, 1e-2f);
}

TEST_F(TestStateFeedbackController, lqr_control_matches_dare_reference_for_velocity_state)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 0.0f }, { 1.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -3.5747f, 1e-2f);
}

TEST_F(TestStateFeedbackController, lqr_precomputed_gain_agrees_with_dare_gain)
{
    controllers::Lqr<float, 2, 1> lqrDare{ A, B, Q, R };
    const auto& K = lqrDare.GetGain();

    controllers::Lqr<float, 2, 1> lqrPre{ K };

    math::Vector<float, 2> state{ { 1.0f }, { 1.0f } };
    auto uDare = lqrDare.ComputeControl(state);
    auto uPre = lqrPre.ComputeControl(state);

    EXPECT_NEAR(uPre.at(0, 0), uDare.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestStateFeedbackController, lqr_input_vector_type_matches_interface)
{
    static_assert(
        std::is_same_v<
            controllers::Lqr<float, 2, 1>::InputVector,
            controllers::StateFeedbackController<float, 2, 1>::InputVector>,
        "Lqr::InputVector must match the interface type");
}

TEST_F(TestStateFeedbackController, lqr_deterministic_same_state_same_output)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 1.0f }, { 0.5f } };
    auto u1 = lqr.ComputeControl(state);
    auto u2 = lqr.ComputeControl(state);

    EXPECT_FLOAT_EQ(u1.at(0, 0), u2.at(0, 0));
}

TEST_F(TestStateFeedbackController, mpc_compute_control_returns_nonzero_for_nonzero_state)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NE(u.at(0, 0), 0.0f);
}

TEST_F(TestStateFeedbackController, mpc_zero_state_produces_zero_control)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> state{ { 0.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestStateFeedbackController, mpc_control_matches_reference_for_position_state)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -0.5877f, 1e-3f);
}

TEST_F(TestStateFeedbackController, mpc_negative_state_produces_positive_control)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> state{ { -1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), 0.5877f, 1e-3f);
}

TEST_F(TestStateFeedbackController, mpc_umin_constraint_clamps_negative_control)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    math::Vector<float, 1> uMin{ { -0.3f } };
    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMin = uMin;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights, constraints };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -0.3f, 1e-4f);
}

TEST_F(TestStateFeedbackController, mpc_umax_constraint_clamps_positive_control)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    math::Vector<float, 1> uMax{ { 0.2f } };
    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMax = uMax;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights, constraints };

    math::Vector<float, 2> state{ { -1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), 0.2f, 1e-4f);
}

TEST_F(TestStateFeedbackController, mpc_set_reference_shifts_control_to_zero_at_reference)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> ref{ { 1.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto u = mpc.ComputeControl(ref);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestStateFeedbackController, mpc_clear_reference_restores_unshifted_control)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> ref{ { 1.0f }, { 0.0f } };
    mpc.SetReference(ref);
    mpc.ClearReference();

    math::Vector<float, 2> state{ { 0.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestStateFeedbackController, mpc_input_vector_type_matches_interface)
{
    static_assert(
        std::is_same_v<
            controllers::Mpc<float, 2, 1, 5, 5>::InputVector,
            controllers::StateFeedbackController<float, 2, 1>::InputVector>,
        "Mpc::InputVector must match the interface type");
}

TEST_F(TestStateFeedbackController, mpc_deterministic_same_state_same_output)
{
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    controllers::Mpc<float, 2, 1, 5, 5> mpc{ A, B, weights };

    math::Vector<float, 2> state{ { 1.0f }, { 0.5f } };
    auto u1 = mpc.ComputeControl(state);
    auto u2 = mpc.ComputeControl(state);

    EXPECT_FLOAT_EQ(u1.at(0, 0), u2.at(0, 0));
}
