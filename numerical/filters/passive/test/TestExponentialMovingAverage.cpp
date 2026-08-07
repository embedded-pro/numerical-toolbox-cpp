#include "numerical/filters/passive/ExponentialMovingAverage.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <numbers>
#include <random>

namespace
{
    class TestExponentialMovingAverage
        : public ::testing::Test
    {
    public:
        filters::passive::ExponentialMovingAverage<float> ema{ 0.5f };
    };
}

TEST_F(TestExponentialMovingAverage, impulse_response_decays_geometrically)
{
    constexpr std::array<float, 5> expected{ 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f };
    constexpr std::array<float, 5> inputs{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    for (std::size_t i = 0; i < expected.size(); ++i)
        EXPECT_NEAR(ema.Filter(inputs[i]), expected[i], 1e-6f);
}

TEST_F(TestExponentialMovingAverage, step_response_matches_analytic_formula)
{
    filters::passive::ExponentialMovingAverage filter{ 0.1f };
    constexpr int steps = 10;
    constexpr float expected = 1.0f - 0.9f * 0.9f * 0.9f * 0.9f * 0.9f * 0.9f * 0.9f * 0.9f * 0.9f * 0.9f;

    float output{};
    for (int i = 0; i < steps; ++i)
        output = filter.Filter(1.0f);

    EXPECT_NEAR(output, expected, math::Tolerance<float>());
}

TEST_F(TestExponentialMovingAverage, dc_gain_is_unity)
{
    filters::passive::ExponentialMovingAverage filter{ 0.25f };
    constexpr float c = 0.8f;

    for (int i = 0; i < 100; ++i)
        filter.Filter(c);

    EXPECT_NEAR(filter.Filter(c), c, math::Tolerance<float>());
}

TEST_F(TestExponentialMovingAverage, nyquist_attenuation_matches_analytic_transfer_function)
{
    constexpr float alpha = 0.5f;
    filters::passive::ExponentialMovingAverage filter{ alpha };
    constexpr float expected = alpha / (2.0f - alpha);

    constexpr int periods = 200;
    constexpr int measurePeriods = 50;
    float sumSqIn{};
    float sumSqOut{};

    for (int i = 0; i < periods; ++i)
    {
        const float x = (i % 2 == 0) ? 1.0f : -1.0f;
        const float y = filter.Filter(x);
        if (i >= periods - measurePeriods)
        {
            sumSqIn += x * x;
            sumSqOut += y * y;
        }
    }

    const float rmsRatio = std::sqrt(sumSqOut / sumSqIn);
    EXPECT_NEAR(rmsRatio, expected, math::Tolerance<float>());
}

TEST_F(TestExponentialMovingAverage, alpha_one_is_passthrough)
{
    filters::passive::ExponentialMovingAverage filter{ 1.0f };
    constexpr std::array<float, 4> inputs{ 0.3f, -0.7f, 1.0f, 0.0f };

    for (const float x : inputs)
        EXPECT_NEAR(filter.Filter(x), x, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, zero_input_from_zero_state_produces_zero_output)
{
    for (int i = 0; i < 10; ++i)
        EXPECT_NEAR(ema.Filter(0.0f), 0.0f, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, constructor_initial_value_sets_state)
{
    constexpr float init = 3.0f;
    filters::passive::ExponentialMovingAverage filter{ 0.5f, init };
    constexpr float expected = init + 0.5f * (0.0f - init);
    EXPECT_NEAR(filter.Filter(0.0f), expected, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, reset_to_zero_restores_fresh_instance_behavior)
{
    ema.Filter(1.0f);
    ema.Filter(1.0f);
    ema.Reset(0.0f);

    filters::passive::ExponentialMovingAverage fresh{ 0.5f };

    constexpr std::array<float, 4> seq{ 0.2f, 0.5f, 0.9f, -0.3f };
    for (const float x : seq)
        EXPECT_FLOAT_EQ(ema.Filter(x), fresh.Filter(x));
}

TEST_F(TestExponentialMovingAverage, reset_to_nonzero_preloads_state)
{
    constexpr float preload = 2.0f;
    ema.Reset(preload);
    constexpr float expected = preload + 0.5f * (0.0f - preload);
    EXPECT_NEAR(ema.Filter(0.0f), expected, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, set_alpha_changes_smoothing_coefficient)
{
    ema.Filter(1.0f);
    ema.SetAlpha(1.0f);
    EXPECT_NEAR(ema.Filter(0.0f), 0.0f, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, determinism_same_input_sequence_identical_output)
{
    filters::passive::ExponentialMovingAverage filter1{ 0.3f };
    filters::passive::ExponentialMovingAverage filter2{ 0.3f };

    constexpr std::array<float, 8> seq{ 0.1f, 0.4f, 0.9f, 0.2f, 0.7f, 0.3f, 0.5f, 0.8f };
    for (const float x : seq)
        EXPECT_FLOAT_EQ(filter1.Filter(x), filter2.Filter(x));
}

TEST_F(TestExponentialMovingAverage, linearity_superposition_from_zero_state)
{
    constexpr float a = 2.0f;
    constexpr float b = -0.5f;
    constexpr float alpha = 0.3f;

    std::mt19937 rng{ 42u };
    std::uniform_real_distribution<float> dist{ -1.0f, 1.0f };

    constexpr int n = 16;
    std::array<float, n> xSeq{};
    std::array<float, n> ySeq{};
    for (int i = 0; i < n; ++i)
    {
        xSeq[i] = dist(rng);
        ySeq[i] = dist(rng);
    }

    filters::passive::ExponentialMovingAverage fxy{ alpha };
    filters::passive::ExponentialMovingAverage fx{ alpha };
    filters::passive::ExponentialMovingAverage fy{ alpha };

    for (int i = 0; i < n; ++i)
    {
        const float combined = fxy.Filter(a * xSeq[i] + b * ySeq[i]);
        const float scaled = a * fx.Filter(xSeq[i]) + b * fy.Filter(ySeq[i]);
        EXPECT_NEAR(combined, scaled, math::Tolerance<float>());
    }
}

TEST_F(TestExponentialMovingAverage, bibo_stability_bounded_input_bounded_output)
{
    filters::passive::ExponentialMovingAverage filter{ 0.05f };
    constexpr float bound = 1.0f;

    std::mt19937 rng{ 7u };
    std::uniform_real_distribution<float> dist{ -bound, bound };

    for (int i = 0; i < 10000; ++i)
    {
        const float out = filter.Filter(dist(rng));
        EXPECT_TRUE(std::isfinite(out));
        EXPECT_LE(std::abs(out), bound + math::Tolerance<float>());
    }
}

TEST_F(TestExponentialMovingAverage, alpha_from_cutoff_matches_analytic_formula)
{
    constexpr float fc = 100.0f;
    constexpr float fs = 1000.0f;
    constexpr float dt = 1.0f / fs;
    constexpr float rc = 1.0f / (2.0f * std::numbers::pi_v<float> * fc);
    constexpr float expected = dt / (rc + dt);

    const float alpha = filters::passive::ExponentialMovingAverage<float>::AlphaFromCutoff(fc, fs);
    EXPECT_NEAR(alpha, expected, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, alpha_from_cutoff_result_in_valid_range)
{
    const float alpha = filters::passive::ExponentialMovingAverage<float>::AlphaFromCutoff(100.0f, 1000.0f);
    EXPECT_GT(alpha, 0.0f);
    EXPECT_LE(alpha, 1.0f);
}
