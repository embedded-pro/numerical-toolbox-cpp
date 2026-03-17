#include "numerical/controllers/implementations/Lqr.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestLqr : public ::testing::Test
    {
    };
}

TEST_F(TestLqr, compute_control_returns_negative_gain_times_state)
{
    math::Matrix<float, 1, 2> K{
        { 0.5f, 0.3f }
    };

    controllers::Lqr<float, 2, 1> lqr(K);

    math::Vector<float, 2> state{ { 1.0f }, { 2.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -1.1f, 1e-3f);
}

TEST_F(TestLqr, compute_control_with_zero_state_returns_zero)
{
    math::Matrix<float, 1, 2> K{
        { 0.5f, 0.3f }
    };

    controllers::Lqr<float, 2, 1> lqr(K);

    math::Vector<float, 2> zeroState{ { 0.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(zeroState);

    EXPECT_NEAR(u.at(0, 0), 0.0f, 1e-6f);
}

TEST_F(TestLqr, compute_control_with_multi_input)
{
    math::Matrix<float, 2, 2> K{
        { 1.0f, 0.0f },
        { 0.0f, 2.0f }
    };

    controllers::Lqr<float, 2, 2> lqr(K);

    math::Vector<float, 2> state{ { 3.0f }, { 4.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -3.0f, 1e-3f);
    EXPECT_NEAR(u.at(1, 0), -8.0f, 1e-3f);
}

TEST_F(TestLqr, get_gain_returns_precomputed_gain)
{
    math::Matrix<float, 1, 2> K{
        { 0.5f, 0.3f }
    };

    controllers::Lqr<float, 2, 1> lqr(K);

    EXPECT_NEAR(lqr.GetGain().at(0, 0), 0.5f, 1e-6f);
    EXPECT_NEAR(lqr.GetGain().at(0, 1), 0.3f, 1e-6f);
}

TEST_F(TestLqr, dare_constructor_produces_non_zero_gain)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr(A, B, Q, R);

    EXPECT_GT(lqr.GetGain().at(0, 0), 0.0f);
}

TEST_F(TestLqr, dare_constructor_stores_riccati_solution)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr(A, B, Q, R);

    EXPECT_GT(lqr.GetRiccatiSolution().at(0, 0), 0.0f);
}
