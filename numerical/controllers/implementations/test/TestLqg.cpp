#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqg.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestLqg : public ::testing::Test
    {
    protected:
        static constexpr std::size_t kState = 2;
        static constexpr std::size_t kInput = 1;
        static constexpr std::size_t kMeas = 1;

        static constexpr float kK0 = 7.604472f;
        static constexpr float kK1 = 4.977648f;

        math::SquareMatrix<float, kState> A{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, kState, kInput> B{
            { 0.0f },
            { 0.1f }
        };
        math::Matrix<float, kMeas, kState> C{ { 1.0f, 0.0f } };
        math::Matrix<float, kMeas, kInput> D{};

        math::LinearTimeInvariant<float, kState, kInput, kMeas> plant{ A, B, C, D };

        controllers::LqgWeights<float, kState, kInput> weights{
            math::SquareMatrix<float, kState>{ { 10.0f, 0.0f }, { 0.0f, 1.0f } },
            math::SquareMatrix<float, kInput>{ { 0.1f } }
        };

        controllers::LqgNoise<float, kState, kInput, kMeas> noise{
            math::SquareMatrix<float, kState>{ { 0.01f, 0.0f }, { 0.0f, 0.01f } },
            math::SquareMatrix<float, kMeas>{ { 0.1f } }
        };

        math::Vector<float, kState> initialState{};
        math::SquareMatrix<float, kState> initialCovariance{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };

        controllers::Lqg<float, kState, kInput, kMeas> lqg{
            plant, noise, weights, initialState, initialCovariance
        };

        controllers::Lqg<float, kState, kInput, kMeas> MakeFreshLqg() const
        {
            return { plant, noise, weights, initialState, initialCovariance };
        }
    };
}

TEST_F(TestLqg, lqr_gain_matches_independent_dare_reference)
{
    EXPECT_NEAR(lqg.GetGain().at(0, 0), kK0, 5e-2f);
    EXPECT_NEAR(lqg.GetGain().at(0, 1), kK1, 5e-2f);
}

TEST_F(TestLqg, estimated_state_starts_at_constructor_initial_state)
{
    auto x = lqg.GetEstimatedState();
    EXPECT_NEAR(x.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestLqg, control_output_is_finite_for_zero_measurement)
{
    math::Vector<float, kMeas> z{};
    const auto u = lqg.ComputeControl(z);
    EXPECT_TRUE(std::isfinite(u.at(0, 0)));
}

TEST_F(TestLqg, two_fresh_instances_produce_identical_first_control_output)
{
    auto lqg1 = MakeFreshLqg();
    auto lqg2 = MakeFreshLqg();

    math::Vector<float, kMeas> z{ { 1.0f } };
    const float u1 = lqg1.ComputeControl(z).at(0, 0);
    const float u2 = lqg2.ComputeControl(z).at(0, 0);

    EXPECT_FLOAT_EQ(u1, u2);
}

TEST_F(TestLqg, estimated_state_updates_after_measurement)
{
    math::Vector<float, kMeas> z{ { 1.0f } };
    lqg.ComputeControl(z);

    const auto x = lqg.GetEstimatedState();
    const float norm = std::sqrt(x.at(0, 0) * x.at(0, 0) + x.at(1, 0) * x.at(1, 0));
    EXPECT_GT(norm, math::Tolerance<float>());
}

TEST_F(TestLqg, noiseless_simulation_state_norm_decays_over_horizon)
{
    math::Vector<float, kState> trueState{ { 1.0f }, { 0.0f } };

    const float normInit = std::sqrt(
        trueState.at(0, 0) * trueState.at(0, 0) +
        trueState.at(1, 0) * trueState.at(1, 0));

    for (std::size_t step = 0; step < 100; ++step)
    {
        math::Vector<float, kMeas> z{ { trueState.at(0, 0) } };
        const auto u = lqg.ComputeControl(z);

        const float x0 = A.at(0, 0) * trueState.at(0, 0) + A.at(0, 1) * trueState.at(1, 0) +
                          B.at(0, 0) * u.at(0, 0);
        const float x1 = A.at(1, 0) * trueState.at(0, 0) + A.at(1, 1) * trueState.at(1, 0) +
                          B.at(1, 0) * u.at(0, 0);
        trueState = math::Vector<float, kState>{ { x0 }, { x1 } };
    }

    const float normFinal = std::sqrt(
        trueState.at(0, 0) * trueState.at(0, 0) +
        trueState.at(1, 0) * trueState.at(1, 0));

    EXPECT_LT(normFinal, normInit * 1e-3f);
}

TEST_F(TestLqg, polymorphic_interface_computes_same_output_as_concrete)
{
    auto lqg1 = MakeFreshLqg();
    auto lqg2 = MakeFreshLqg();

    math::Vector<float, kMeas> z{ { 0.5f } };
    const float uConcrete = lqg1.ComputeControl(z).at(0, 0);

    controllers::OutputFeedbackController<float, kState, kInput, kMeas>* ptr = &lqg2;
    const float uInterface = ptr->ComputeControl(z).at(0, 0);

    EXPECT_FLOAT_EQ(uConcrete, uInterface);
}
