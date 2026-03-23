#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestMpc : public ::testing::Test
    {
    };
}

TEST_F(TestMpc, compute_control_returns_zero_for_zero_state)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 5, 5> mpc(A, B, weights);

    math::Vector<float, 2> zeroState{ { 0.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(zeroState);

    EXPECT_NEAR(u.at(0, 0), 0.0f, 1e-4f);
}

TEST_F(TestMpc, compute_control_produces_nonzero_for_nonzero_state)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 5, 5> mpc(A, B, weights);

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_NE(u.at(0, 0), 0.0f);
}

TEST_F(TestMpc, compute_control_drives_state_toward_origin)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 10.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 10> mpc(A, B, weights);

    math::Vector<float, 2> state{ { 5.0f }, { 0.0f } };

    for (int step = 0; step < 50; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 0.0f, 0.1f);
    EXPECT_NEAR(state.at(1, 0), 0.0f, 0.1f);
}

TEST_F(TestMpc, control_sequence_length_equals_control_horizon)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 5> mpc(A, B, weights);

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };
    mpc.ComputeControl(state);

    auto seq = mpc.GetControlSequence();
    EXPECT_EQ(seq.size(), 5u);
}

TEST_F(TestMpc, box_constraints_clamp_control_input)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 100.0f, 0.0f },
        { 0.0f, 100.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.01f }
    };

    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMin = math::Vector<float, 1>{ { -0.5f } };
    constraints.uMax = math::Vector<float, 1>{ { 0.5f } };

    controllers::Mpc<float, 2, 1, 5, 5> mpc(A, B, weights, constraints);

    math::Vector<float, 2> state{ { 10.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_GE(u.at(0, 0), -0.5f);
    EXPECT_LE(u.at(0, 0), 0.5f);

    auto seq = mpc.GetControlSequence();
    for (std::size_t k = 0; k < seq.size(); ++k)
    {
        EXPECT_GE(seq[k].at(0, 0), -0.5f);
        EXPECT_LE(seq[k].at(0, 0), 0.5f);
    }
}

TEST_F(TestMpc, precomputed_constructor_matches_online_computation)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 5, 5> mpcOnline(A, B, weights);

    auto H = mpcOnline.GetHessian();
    auto F = mpcOnline.GetGradientMatrix();

    controllers::Mpc<float, 2, 1, 5, 5> mpcPrecomputed(H, F);

    math::Vector<float, 2> state{ { 2.0f }, { -1.0f } };
    auto uOnline = mpcOnline.ComputeControl(state);
    auto uPrecomputed = mpcPrecomputed.ComputeControl(state);

    EXPECT_NEAR(uOnline.at(0, 0), uPrecomputed.at(0, 0), 1e-4f);
}

TEST_F(TestMpc, double_integrator_regulation)
{
    // Double integrator: x1 = position, x2 = velocity
    // x1[k+1] = x1[k] + dt * x2[k]
    // x2[k+1] = x2[k] + dt * u[k]
    float dt = 0.1f;

    math::SquareMatrix<float, 2> A{
        { 1.0f, dt },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f * dt * dt },
        { dt }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 10> mpc(A, B, weights);

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };

    float prevPosition = math::ToFloat(state.at(0, 0));

    for (int step = 0; step < 100; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 0.0f, 0.01f);
    EXPECT_NEAR(state.at(1, 0), 0.0f, 0.01f);
}

TEST_F(TestMpc, unconstrained_mpc_approaches_lqr_with_long_horizon)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    math::SquareMatrix<float, 2> Q{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    math::SquareMatrix<float, 1> R{
        { 0.1f }
    };

    // MPC with DARE terminal cost and long horizon should approximate LQR
    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = Q;
    weights.R = R;

    // Compute terminal cost via DARE
    solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> dare;
    auto P = dare.Solve(A, B, Q, R);
    weights.terminalP = P;

    controllers::Mpc<float, 2, 1, 10, 10> mpc(A, B, weights);
    controllers::Lqr<float, 2, 1> lqr(A, B, Q, R);

    math::Vector<float, 2> state{ { 1.0f }, { 0.5f } };

    auto uMpc = mpc.ComputeControl(state);
    auto uLqr = lqr.ComputeControl(state);

    // MPC with DARE terminal cost and sufficient horizon should be close to LQR
    EXPECT_NEAR(math::ToFloat(uMpc.at(0, 0)), math::ToFloat(uLqr.at(0, 0)), 0.1f);
}

TEST_F(TestMpc, reference_tracking_drives_state_to_reference)
{
    float dt = 0.1f;

    math::SquareMatrix<float, 2> A{
        { 1.0f, dt },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f * dt * dt },
        { dt }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 10> mpc(A, B, weights);

    math::Vector<float, 2> ref{ { 3.0f }, { 0.0f } };
    mpc.SetReference(ref);

    math::Vector<float, 2> state{ { 0.0f }, { 0.0f } };

    for (int step = 0; step < 200; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 3.0f, 0.05f);
    EXPECT_NEAR(state.at(1, 0), 0.0f, 0.05f);
}

TEST_F(TestMpc, clear_reference_reverts_to_origin_regulation)
{
    float dt = 0.1f;

    math::SquareMatrix<float, 2> A{
        { 1.0f, dt },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.5f * dt * dt },
        { dt }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 10> mpc(A, B, weights);

    auto ref = math::Vector<float, 2>{ { 5.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto state = math::Vector<float, 2>{ { 0.0f }, { 0.0f } };
    auto u1 = mpc.ComputeControl(state);
    EXPECT_GT(u1.at(0, 0), 0.0f);

    mpc.ClearReference();

    auto u2 = mpc.ComputeControl(state);
    EXPECT_NEAR(u2.at(0, 0), 0.0f, 1e-4f);
}

TEST_F(TestMpc, three_state_system_regulation)
{
    math::SquareMatrix<float, 3> A{
        { 1.0f, 0.1f, 0.0f },
        { 0.0f, 1.0f, 0.1f },
        { 0.0f, 0.0f, 1.0f }
    };
    math::Matrix<float, 3, 1> B{
        { 0.0f },
        { 0.0f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 3, 1> weights;
    weights.Q = math::SquareMatrix<float, 3>{
        { 10.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 3, 1, 10, 10> mpc(A, B, weights);

    auto state = math::Vector<float, 3>{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int step = 0; step < 200; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 0.0f, 0.1f);
    EXPECT_NEAR(state.at(1, 0), 0.0f, 0.1f);
    EXPECT_NEAR(state.at(2, 0), 0.0f, 0.1f);
}

TEST_F(TestMpc, three_state_system_with_constraints)
{
    math::SquareMatrix<float, 3> A{
        { 1.0f, 0.1f, 0.0f },
        { 0.0f, 1.0f, 0.1f },
        { 0.0f, 0.0f, 1.0f }
    };
    math::Matrix<float, 3, 1> B{
        { 0.0f },
        { 0.0f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 3, 1> weights;
    weights.Q = math::SquareMatrix<float, 3>{
        { 10.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMin = math::Vector<float, 1>{ { -0.3f } };
    constraints.uMax = math::Vector<float, 1>{ { 0.3f } };

    controllers::Mpc<float, 3, 1, 10, 10> mpc(A, B, weights, constraints);

    auto state = math::Vector<float, 3>{ { 2.0f }, { 0.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_GE(u.at(0, 0), -0.3f);
    EXPECT_LE(u.at(0, 0), 0.3f);
}

TEST_F(TestMpc, three_state_system_reference_tracking)
{
    math::SquareMatrix<float, 3> A{
        { 1.0f, 0.1f, 0.0f },
        { 0.0f, 1.0f, 0.1f },
        { 0.0f, 0.0f, 1.0f }
    };
    math::Matrix<float, 3, 1> B{
        { 0.0f },
        { 0.0f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 3, 1> weights;
    weights.Q = math::SquareMatrix<float, 3>{
        { 10.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 3, 1, 10, 10> mpc(A, B, weights);

    auto ref = math::Vector<float, 3>{ { 2.0f }, { 0.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto state = math::Vector<float, 3>{ { 0.0f }, { 0.0f }, { 0.0f } };

    for (int step = 0; step < 300; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 2.0f, 0.2f);
}

TEST_F(TestMpc, three_state_precomputed_constructor)
{
    math::SquareMatrix<float, 3> A{
        { 1.0f, 0.1f, 0.0f },
        { 0.0f, 1.0f, 0.1f },
        { 0.0f, 0.0f, 1.0f }
    };
    math::Matrix<float, 3, 1> B{
        { 0.0f },
        { 0.0f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 3, 1> weights;
    weights.Q = math::SquareMatrix<float, 3>{
        { 10.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 3, 1, 10, 10> mpcOnline(A, B, weights);

    auto H = mpcOnline.GetHessian();
    auto F = mpcOnline.GetGradientMatrix();

    controllers::Mpc<float, 3, 1, 10, 10> mpcPrecomputed(H, F);

    auto state = math::Vector<float, 3>{ { 1.0f }, { -0.5f }, { 0.2f } };
    auto uOnline = mpcOnline.ComputeControl(state);
    auto uPrecomputed = mpcPrecomputed.ComputeControl(state);

    EXPECT_NEAR(uOnline.at(0, 0), uPrecomputed.at(0, 0), 1e-3f);
}

TEST_F(TestMpc, different_prediction_and_control_horizons_with_constraints)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMin = math::Vector<float, 1>{ { -1.0f } };
    constraints.uMax = math::Vector<float, 1>{ { 1.0f } };

    controllers::Mpc<float, 2, 1, 10, 5> mpc(A, B, weights, constraints);

    auto state = math::Vector<float, 2>{ { 5.0f }, { 0.0f } };
    auto u = mpc.ComputeControl(state);

    EXPECT_GE(u.at(0, 0), -1.0f);
    EXPECT_LE(u.at(0, 0), 1.0f);

    auto seq = mpc.GetControlSequence();
    EXPECT_EQ(seq.size(), 5u);
}

TEST_F(TestMpc, different_prediction_and_control_horizons_reference_tracking)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 5> mpc(A, B, weights);

    auto ref = math::Vector<float, 2>{ { 2.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto state = math::Vector<float, 2>{ { 0.0f }, { 0.0f } };

    for (int step = 0; step < 200; ++step)
    {
        auto u = mpc.ComputeControl(state);
        state = A * state + B * u;
    }

    EXPECT_NEAR(state.at(0, 0), 2.0f, 0.1f);
}

TEST_F(TestMpc, different_prediction_and_control_horizons_precomputed)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 5> mpcOnline(A, B, weights);

    auto H = mpcOnline.GetHessian();
    auto F = mpcOnline.GetGradientMatrix();

    controllers::Mpc<float, 2, 1, 10, 5> mpcPrecomputed(H, F);

    auto state = math::Vector<float, 2>{ { 3.0f }, { -1.0f } };
    auto uOnline = mpcOnline.ComputeControl(state);
    auto uPrecomputed = mpcPrecomputed.ComputeControl(state);

    EXPECT_NEAR(uOnline.at(0, 0), uPrecomputed.at(0, 0), 1e-3f);
}

TEST_F(TestMpc, ten_horizon_precomputed_with_constraints)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 10> mpcOnline(A, B, weights);

    auto H = mpcOnline.GetHessian();
    auto F = mpcOnline.GetGradientMatrix();

    controllers::MpcConstraints<float, 1> constraints;
    constraints.uMin = math::Vector<float, 1>{ { -0.5f } };
    constraints.uMax = math::Vector<float, 1>{ { 0.5f } };

    controllers::Mpc<float, 2, 1, 10, 10> mpcPrecomputed(H, F, constraints);

    auto state = math::Vector<float, 2>{ { 5.0f }, { -1.0f } };
    auto u = mpcPrecomputed.ComputeControl(state);

    EXPECT_GE(u.at(0, 0), -0.5f);
    EXPECT_LE(u.at(0, 0), 0.5f);

    auto seq = mpcPrecomputed.GetControlSequence();
    EXPECT_EQ(seq.size(), 10u);
}

TEST_F(TestMpc, three_state_clear_reference_and_get_sequence)
{
    math::SquareMatrix<float, 3> A{
        { 1.0f, 0.1f, 0.0f },
        { 0.0f, 1.0f, 0.1f },
        { 0.0f, 0.0f, 1.0f }
    };
    math::Matrix<float, 3, 1> B{
        { 0.0f },
        { 0.0f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 3, 1> weights;
    weights.Q = math::SquareMatrix<float, 3>{
        { 10.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 3, 1, 10, 10> mpc(A, B, weights);

    auto ref = math::Vector<float, 3>{ { 1.0f }, { 0.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto state = math::Vector<float, 3>{ { 0.0f }, { 0.0f }, { 0.0f } };
    auto u1 = mpc.ComputeControl(state);
    EXPECT_NE(u1.at(0, 0), 0.0f);

    mpc.ClearReference();

    auto u2 = mpc.ComputeControl(state);
    EXPECT_NEAR(u2.at(0, 0), 0.0f, 1e-4f);

    auto seq = mpc.GetControlSequence();
    EXPECT_EQ(seq.size(), 10u);
}

TEST_F(TestMpc, different_horizons_clear_reference)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.1f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.005f },
        { 0.1f }
    };

    controllers::MpcWeights<float, 2, 1> weights;
    weights.Q = math::SquareMatrix<float, 2>{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    weights.R = math::SquareMatrix<float, 1>{
        { 0.1f }
    };

    controllers::Mpc<float, 2, 1, 10, 5> mpc(A, B, weights);

    auto ref = math::Vector<float, 2>{ { 5.0f }, { 0.0f } };
    mpc.SetReference(ref);

    auto state = math::Vector<float, 2>{ { 0.0f }, { 0.0f } };
    auto u1 = mpc.ComputeControl(state);
    EXPECT_GT(u1.at(0, 0), 0.0f);

    mpc.ClearReference();

    auto u2 = mpc.ComputeControl(state);
    EXPECT_NEAR(u2.at(0, 0), 0.0f, 1e-4f);
}
