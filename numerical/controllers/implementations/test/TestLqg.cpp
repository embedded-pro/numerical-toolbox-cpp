#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqg.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestLqg : public ::testing::Test
    {
    protected:
        static constexpr std::size_t stateSize = 2;
        static constexpr std::size_t inputSize = 1;
        static constexpr std::size_t measurementSize = 1;

        math::SquareMatrix<float, stateSize> A{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, stateSize, inputSize> B{
            { 0.0f },
            { 0.1f }
        };
        math::Matrix<float, measurementSize, stateSize> C{
            { 1.0f, 0.0f }
        };
        math::Matrix<float, measurementSize, inputSize> D{};

        math::LinearTimeInvariant<float, stateSize, inputSize, measurementSize> plant{ A, B, C, D };

        controllers::LqgWeights<float, stateSize, inputSize> weights{
            math::SquareMatrix<float, stateSize>{ { 10.0f, 0.0f }, { 0.0f, 1.0f } },
            math::SquareMatrix<float, inputSize>{ { 0.1f } }
        };

        controllers::LqgNoise<float, stateSize, inputSize, measurementSize> noise{
            math::SquareMatrix<float, stateSize>{ { 0.01f, 0.0f }, { 0.0f, 0.01f } },
            math::SquareMatrix<float, measurementSize>{ { 0.1f } }
        };

        math::Vector<float, stateSize> initialState{};
        math::SquareMatrix<float, stateSize> initialCovariance{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };

        controllers::Lqg<float, stateSize, inputSize, measurementSize> lqg{
            plant, noise, weights, initialState, initialCovariance
        };
    };
}

TEST_F(TestLqg, compute_control_returns_finite_value_for_zero_measurement)
{
    math::Vector<float, measurementSize> z{};
    auto u = lqg.ComputeControl(z);
    EXPECT_TRUE(std::isfinite(u.at(0, 0)));
}

TEST_F(TestLqg, get_gain_returns_nonzero_lqr_gain)
{
    const auto& K = lqg.GetGain();
    float normSq = K.at(0, 0) * K.at(0, 0) + K.at(0, 1) * K.at(0, 1);
    EXPECT_GT(normSq, 0.0f);
}

TEST_F(TestLqg, get_estimated_state_starts_at_initial_state)
{
    auto x = lqg.GetEstimatedState();
    EXPECT_NEAR(x.at(0, 0), initialState.at(0, 0), 1e-6f);
    EXPECT_NEAR(x.at(1, 0), initialState.at(1, 0), 1e-6f);
}

TEST_F(TestLqg, repeated_calls_update_estimated_state)
{
    math::Vector<float, measurementSize> z{ { 1.0f } };
    lqg.ComputeControl(z);
    lqg.ComputeControl(z);

    auto x = lqg.GetEstimatedState();
    EXPECT_NE(x.at(0, 0), initialState.at(0, 0));
}

TEST_F(TestLqg, control_drives_state_toward_zero)
{
    math::Vector<float, measurementSize> z{ { 5.0f } };

    float firstU = lqg.ComputeControl(z).at(0, 0);

    math::Vector<float, measurementSize> zNear{ { 0.1f } };
    float laterU = lqg.ComputeControl(zNear).at(0, 0);

    EXPECT_LT(std::abs(laterU), std::abs(firstU) + 1.0f);
}

TEST_F(TestLqg, lqg_satisfies_output_feedback_controller_interface)
{
    controllers::OutputFeedbackController<float, stateSize, inputSize, measurementSize>* ptr = &lqg;
    math::Vector<float, measurementSize> z{};
    auto u = ptr->ComputeControl(z);
    EXPECT_TRUE(std::isfinite(u.at(0, 0)));
}
