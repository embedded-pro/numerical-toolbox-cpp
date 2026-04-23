#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include "numerical/controllers/interfaces/StateFeedbackController.hpp"
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

TEST_F(TestStateFeedbackController, lqr_compute_control_returns_nonzero_for_nonzero_state)
{
    controllers::Lqr<float, 2, 1> lqr{ A, B, Q, R };

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NE(u.at(0, 0), 0.0f);
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
