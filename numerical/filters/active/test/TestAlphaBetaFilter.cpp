#include "numerical/filters/active/AlphaBetaFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestAlphaBetaFilter
        : public ::testing::Test
    {
    protected:
        filters::AlphaBetaFilter<float, 2> filter{ 0.5f, 0.1f, 1.0f };
    };

    class TestAlphaBetaGammaFilter
        : public ::testing::Test
    {
    protected:
        filters::AlphaBetaFilter<float, 3> filter{ 0.5f, 0.1f, 0.01f, 1.0f };
    };
}

TEST_F(TestAlphaBetaFilter, first_sample_seeds_position)
{
    float result{ filter.Filter(0.4f) };

    EXPECT_NEAR(result, 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaFilter, constant_position_zero_velocity)
{
    for (int i = 0; i < 50; ++i)
        filter.Filter(0.3f);

    EXPECT_NEAR(filter.State()[0], 0.3f, 1e-3f);
    EXPECT_NEAR(filter.State()[1], 0.0f, 1e-3f);
}

TEST_F(TestAlphaBetaFilter, constant_velocity_tracks_ramp)
{
    constexpr float v0{ 0.2f };
    constexpr float Ts{ 1.0f };

    for (int n = 0; n < 200; ++n)
        filter.Filter(v0 * static_cast<float>(n) * Ts);

    EXPECT_NEAR(filter.State()[1], v0, 1e-2f);
    EXPECT_NEAR(filter.State()[0], v0 * 199.0f * Ts, 1.0f);
}

TEST_F(TestAlphaBetaFilter, step_response_settles)
{
    for (int i = 0; i < 100; ++i)
        filter.Filter(1.0f);

    EXPECT_NEAR(filter.State()[0], 1.0f, 1e-2f);
}

TEST_F(TestAlphaBetaFilter, gains_from_tracking_index_are_stable)
{
    auto g{ filters::AlphaBetaFilter<float, 2>::GainsFromTrackingIndex(0.5f) };

    EXPECT_GT(g.alpha, 0.0f);
    EXPECT_LT(g.alpha, 1.0f);
    EXPECT_GT(g.beta, 0.0f);
    EXPECT_LT(g.beta, 4.0f - 2.0f * g.alpha);
}

TEST_F(TestAlphaBetaFilter, noise_is_attenuated)
{
    constexpr int N{ 200 };
    constexpr float noiseAmplitude{ 0.1f };

    float inputVariance{ 0.0f };
    float outputVariance{ 0.0f };
    float inputMean{ 1.0f };
    float outputMean{ 0.0f };

    std::array<float, N> outputs{};
    for (int i = 0; i < N; ++i)
    {
        float noise{ noiseAmplitude * (i % 2 == 0 ? 1.0f : -1.0f) };
        float meas{ inputMean + noise };
        outputs[static_cast<std::size_t>(i)] = filter.Filter(meas);
    }

    for (int i = N / 2; i < N; ++i)
        outputMean += outputs[static_cast<std::size_t>(i)];
    outputMean /= static_cast<float>(N / 2);

    for (int i = N / 2; i < N; ++i)
    {
        float od{ outputs[static_cast<std::size_t>(i)] - outputMean };
        outputVariance += od * od;
    }
    outputVariance /= static_cast<float>(N / 2);

    inputVariance = noiseAmplitude * noiseAmplitude;

    EXPECT_LT(outputVariance, inputVariance);
}

TEST_F(TestAlphaBetaFilter, reset_clears_state)
{
    for (int i = 0; i < 10; ++i)
        filter.Filter(5.0f);

    filter.Reset(0.0f);

    float result{ filter.Filter(0.7f) };

    EXPECT_NEAR(result, 0.7f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaGammaFilter, alpha_beta_gamma_tracks_acceleration)
{
    constexpr float a0{ 0.2f };
    constexpr float Ts{ 1.0f };

    for (int n = 0; n < 300; ++n)
    {
        float t{ static_cast<float>(n) * Ts };
        filter.Filter(0.5f * a0 * t * t);
    }

    EXPECT_NEAR(filter.State()[2], a0, 1e-1f);
    float t299{ 299.0f * Ts };
    float expectedPos{ 0.5f * a0 * t299 * t299 };
    EXPECT_NEAR(filter.State()[0], expectedPos, expectedPos * 0.05f);
}
