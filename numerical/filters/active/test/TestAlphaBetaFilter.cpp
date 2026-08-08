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

TEST_F(TestAlphaBetaFilter, first_sample_seeds_position_and_zeroes_velocity)
{
    float result{ filter.Filter(0.4f) };

    EXPECT_NEAR(result, 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaFilter, two_step_ramp_matches_hand_computed_reference)
{
    filter.Filter(0.0f);
    float p1{ filter.Filter(0.2f) };

    EXPECT_NEAR(p1, 0.10f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.020f, math::Tolerance<float>());

    float p2{ filter.Filter(0.4f) };

    EXPECT_NEAR(p2, 0.26f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.048f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaFilter, constant_position_converges_to_reference)
{
    for (int i = 0; i < 100; ++i)
        filter.Filter(1.0f);

    EXPECT_NEAR(filter.State()[0], 1.0f, 1e-2f);
    EXPECT_NEAR(filter.State()[1], 0.0f, 1e-2f);
}

TEST_F(TestAlphaBetaFilter, constant_velocity_velocity_estimate_converges)
{
    constexpr float v0{ 0.2f };
    constexpr float Ts{ 1.0f };

    for (int n = 0; n < 200; ++n)
        filter.Filter(v0 * static_cast<float>(n) * Ts);

    EXPECT_NEAR(filter.State()[1], v0, 1e-2f);
}

TEST_F(TestAlphaBetaFilter, gains_from_tracking_index_match_kalata_formula)
{
    auto g{ filters::AlphaBetaFilter<float, 2>::GainsFromTrackingIndex(0.5f) };

    EXPECT_NEAR(g.alpha, 0.62838f, 1e-4f);
    EXPECT_NEAR(g.beta, 0.30478f, 1e-4f);
}

TEST_F(TestAlphaBetaFilter, gains_from_tracking_index_satisfy_stability_conditions)
{
    auto g{ filters::AlphaBetaFilter<float, 2>::GainsFromTrackingIndex(0.5f) };

    EXPECT_GT(g.alpha, 0.0f);
    EXPECT_LT(g.alpha, 1.0f);
    EXPECT_GT(g.beta, 0.0f);
    EXPECT_LT(g.beta, 4.0f - 2.0f * g.alpha);
}

TEST_F(TestAlphaBetaFilter, noise_variance_is_attenuated_below_half_input)
{
    constexpr int N{ 512 };
    constexpr float noiseAmplitude{ 0.1f };
    constexpr float inputMean{ 1.0f };

    std::array<float, N> outputs{};
    for (int i = 0; i < N; ++i)
    {
        float noise{ noiseAmplitude * (i % 2 == 0 ? 1.0f : -1.0f) };
        outputs[static_cast<std::size_t>(i)] = filter.Filter(inputMean + noise);
    }

    float outputMean{ 0.0f };
    for (int i = N / 2; i < N; ++i)
        outputMean += outputs[static_cast<std::size_t>(i)];
    outputMean /= static_cast<float>(N / 2);

    float outputVariance{ 0.0f };
    for (int i = N / 2; i < N; ++i)
    {
        float d{ outputs[static_cast<std::size_t>(i)] - outputMean };
        outputVariance += d * d;
    }
    outputVariance /= static_cast<float>(N / 2);

    float inputVariance{ noiseAmplitude * noiseAmplitude };

    EXPECT_LT(outputVariance, 0.5f * inputVariance);
}

TEST_F(TestAlphaBetaFilter, reset_restores_initial_state_on_subsequent_sample)
{
    for (int i = 0; i < 10; ++i)
        filter.Filter(5.0f);

    filter.Reset(0.0f);
    float result{ filter.Filter(0.7f) };

    EXPECT_NEAR(result, 0.7f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaFilter, two_fresh_instances_produce_identical_outputs)
{
    filters::AlphaBetaFilter<float, 2> filterA{ 0.5f, 0.1f, 1.0f };
    filters::AlphaBetaFilter<float, 2> filterB{ 0.5f, 0.1f, 1.0f };

    constexpr std::array<float, 5> measurements{ 0.0f, 0.2f, 0.4f, 0.6f, 0.8f };
    for (auto z : measurements)
    {
        float outA{ filterA.Filter(z) };
        float outB{ filterB.Filter(z) };
        EXPECT_FLOAT_EQ(outA, outB);
    }
}

TEST_F(TestAlphaBetaFilter, reset_then_reseed_equals_fresh_instance)
{
    for (int i = 0; i < 20; ++i)
        filter.Filter(3.0f);

    filter.Reset(0.0f);

    filters::AlphaBetaFilter<float, 2> fresh{ 0.5f, 0.1f, 1.0f };

    constexpr std::array<float, 4> measurements{ 0.0f, 1.0f, 2.0f, 3.0f };
    for (auto z : measurements)
        EXPECT_FLOAT_EQ(filter.Filter(z), fresh.Filter(z));
}

TEST_F(TestAlphaBetaFilter, zero_input_yields_zero_state)
{
    for (int i = 0; i < 50; ++i)
    {
        float out{ filter.Filter(0.0f) };
        EXPECT_FLOAT_EQ(out, 0.0f);
    }

    EXPECT_FLOAT_EQ(filter.State()[0], 0.0f);
    EXPECT_FLOAT_EQ(filter.State()[1], 0.0f);
}

TEST_F(TestAlphaBetaFilter, output_is_finite_for_large_magnitude_input)
{
    constexpr float bigValue{ 1e6f };
    for (int i = 0; i < 10; ++i)
        filter.Filter(bigValue);

    EXPECT_TRUE(std::isfinite(filter.State()[0]));
    EXPECT_TRUE(std::isfinite(filter.State()[1]));
}

TEST_F(TestAlphaBetaGammaFilter, first_sample_seeds_position_and_zeroes_higher_states)
{
    float result{ filter.Filter(1.5f) };

    EXPECT_NEAR(result, 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[2], 0.0f, math::Tolerance<float>());
}

TEST_F(TestAlphaBetaGammaFilter, acceleration_estimate_converges_on_quadratic_input)
{
    constexpr float a0{ 0.2f };
    constexpr float Ts{ 1.0f };

    for (int n = 0; n < 300; ++n)
    {
        float t{ static_cast<float>(n) * Ts };
        filter.Filter(0.5f * a0 * t * t);
    }

    EXPECT_NEAR(filter.State()[2], a0, 1e-1f);
}

TEST_F(TestAlphaBetaGammaFilter, reset_restores_initial_state_order3)
{
    for (int i = 0; i < 10; ++i)
        filter.Filter(9.0f);

    filter.Reset(0.0f);
    float result{ filter.Filter(2.0f) };

    EXPECT_NEAR(result, 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[1], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(filter.State()[2], 0.0f, math::Tolerance<float>());
}
