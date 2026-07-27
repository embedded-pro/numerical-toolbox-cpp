// Copyright (c) 2025 Gabriel Santos. All rights reserved.
#include "numerical/filters/passive/NotchCombFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TestNotchComb
        : public ::testing::Test
    {
    public:
        filters::passive::NotchFilter<float> notch{ 50.0f, 1000.0f, 10.0f };
        filters::passive::CombFilter<float, 20, false> comb{ 0.9f };
    };
}

TEST_F(TestNotchComb, notch_rejects_target_frequency)
{
    constexpr int settleSamples{ 500 };
    constexpr int measureSamples{ 100 };
    constexpr float fs{ 1000.0f };
    constexpr float f0{ 50.0f };
    const float w0{ 2.0f * std::numbers::pi_v<float> * f0 / fs };

    for (int i = 0; i < settleSamples; ++i)
        notch.Filter(std::sin(static_cast<float>(i) * w0));

    float maxAmp{ 0.0f };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float y{ notch.Filter(std::sin(static_cast<float>(i) * w0)) };
        const float abs_y{ y < 0.0f ? -y : y };
        if (abs_y > maxAmp)
            maxAmp = abs_y;
    }

    EXPECT_NEAR(maxAmp, 0.0f, 0.05f);
}

TEST_F(TestNotchComb, notch_passes_off_frequency)
{
    constexpr int settleSamples{ 200 };
    constexpr int measureSamples{ 100 };
    constexpr float fs{ 1000.0f };
    constexpr float f{ 200.0f };
    const float w{ 2.0f * std::numbers::pi_v<float> * f / fs };

    for (int i = 0; i < settleSamples; ++i)
        notch.Filter(std::sin(static_cast<float>(i) * w));

    float maxAmp{ 0.0f };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float y{ notch.Filter(std::sin(static_cast<float>(i) * w)) };
        const float abs_y{ y < 0.0f ? -y : y };
        if (abs_y > maxAmp)
            maxAmp = abs_y;
    }

    EXPECT_NEAR(maxAmp, 1.0f, 0.05f);
}

TEST_F(TestNotchComb, notch_dc_gain_unity)
{
    constexpr float tol{ 1e-3f };
    constexpr float dc{ 0.7f };
    constexpr int samples{ 200 };

    float last{ 0.0f };
    for (int i = 0; i < samples; ++i)
        last = notch.Filter(dc);

    EXPECT_NEAR(last, dc, tol);
}

TEST_F(TestNotchComb, comb_rejects_harmonics)
{
    constexpr int settleSamples{ 200 };
    constexpr int measureSamples{ 40 };
    constexpr float fs{ 1000.0f };
    constexpr float fundamental{ fs / 20.0f };
    const float w1{ 2.0f * std::numbers::pi_v<float> * fundamental / fs };
    const float w2{ 2.0f * std::numbers::pi_v<float> * 2.0f * fundamental / fs };
    const float w3{ 2.0f * std::numbers::pi_v<float> * 3.0f * fundamental / fs };

    for (int i = 0; i < settleSamples; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w1) +
                       std::sin(static_cast<float>(i) * w2) +
                       std::sin(static_cast<float>(i) * w3) };
        comb.Filter(x);
    }

    float maxAmp{ 0.0f };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w1) +
                       std::sin(static_cast<float>(i) * w2) +
                       std::sin(static_cast<float>(i) * w3) };
        const float y{ comb.Filter(x) };
        const float abs_y{ y < 0.0f ? -y : y };
        if (abs_y > maxAmp)
            maxAmp = abs_y;
    }

    EXPECT_LT(maxAmp, 0.5f);
}

TEST_F(TestNotchComb, feedback_comb_is_stable)
{
    filters::passive::CombFilter<float, 20, true> fbComb{ 0.95f };

    float previous{ fbComb.Filter(1.0f) };
    bool decaying{ true };
    float peakAfterInitial{ 0.0f };

    for (int i = 1; i < 400; ++i)
    {
        const float y{ fbComb.Filter(0.0f) };
        const float abs_y{ y < 0.0f ? -y : y };
        if (abs_y > peakAfterInitial)
            peakAfterInitial = abs_y;
        (void)previous;
        previous = y;
    }

    EXPECT_LT(peakAfterInitial, 100.0f);
}

TEST_F(TestNotchComb, reset_clears_state)
{
    constexpr float tol{ 1e-6f };

    for (int i = 0; i < 50; ++i)
    {
        notch.Filter(1.0f);
        comb.Filter(1.0f);
    }

    notch.Reset();
    comb.Reset();

    EXPECT_NEAR(notch.Filter(0.0f), 0.0f, tol);
    EXPECT_NEAR(comb.Filter(0.0f), 0.0f, tol);
}
