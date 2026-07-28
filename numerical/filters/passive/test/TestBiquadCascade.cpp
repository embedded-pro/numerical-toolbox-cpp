// Copyright (c) 2025 Gabriel Santos. All rights reserved.
#include "numerical/filters/passive/BiquadCascade.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TestBiquadCascade
        : public ::testing::Test
    {
    public:
        static constexpr float fs{ 1000.0f };
        static constexpr float fc{ 100.0f };
        static constexpr float Q{ 0.707f };

        std::array<filters::passive::BiquadCoeffs<float>, 2> lpCoeffs{
            filters::passive::Biquad<float>::LowPass(fc, fs, Q),
            filters::passive::Biquad<float>::LowPass(fc, fs, Q)
        };

        filters::passive::BiquadCascade<float, 2> filter{ lpCoeffs };
    };
}

TEST_F(TestBiquadCascade, dc_gain_of_lowpass_is_unity)
{
    constexpr int settleSamples{ 2000 };
    float last{ 0.0f };
    for (int i = 0; i < settleSamples; ++i)
        last = filter.Filter(1.0f);

    EXPECT_NEAR(last, 1.0f, 1e-3f);
}

TEST_F(TestBiquadCascade, lowpass_attenuates_high_freq)
{
    constexpr float fHigh{ 490.0f };
    const float w{ 2.0f * std::numbers::pi_v<float> * fHigh / fs };
    constexpr int settleSamples{ 500 };
    constexpr int measureSamples{ 100 };

    for (int i = 0; i < settleSamples; ++i)
        filter.Filter(std::sin(static_cast<float>(i) * w));

    float maxAmp{ 0.0f };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float y{ filter.Filter(std::sin(static_cast<float>(i) * w)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_LT(maxAmp, 0.1f);
}

TEST_F(TestBiquadCascade, bypass_coeffs_are_passthrough)
{
    const filters::passive::BiquadCoeffs<float> bypass{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    filters::passive::Biquad<float> bq{ bypass };

    EXPECT_NEAR(bq.Filter(1.0f), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(bq.Filter(0.5f), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(bq.Filter(-2.0f), -2.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, known_impulse_response)
{
    const filters::passive::BiquadCoeffs<float> c{ 0.5f, 0.25f, 0.1f, -0.5f, 0.1f };
    filters::passive::Biquad<float> bq{ c };

    constexpr std::array<float, 5> expected{ 0.5f, 0.5f, 0.3f, 0.1f, 0.02f };
    constexpr float tol{ 1e-5f };

    EXPECT_NEAR(bq.Filter(1.0f), expected[0], tol);
    EXPECT_NEAR(bq.Filter(0.0f), expected[1], tol);
    EXPECT_NEAR(bq.Filter(0.0f), expected[2], tol);
    EXPECT_NEAR(bq.Filter(0.0f), expected[3], tol);
    EXPECT_NEAR(bq.Filter(0.0f), expected[4], tol);
}

TEST_F(TestBiquadCascade, cascade_equals_serial_sections)
{
    const filters::passive::BiquadCoeffs<float> c0{ filters::passive::Biquad<float>::LowPass(fc, fs, Q) };
    const filters::passive::BiquadCoeffs<float> c1{ filters::passive::Biquad<float>::HighPass(200.0f, fs, Q) };

    filters::passive::Biquad<float> s0{ c0 };
    filters::passive::Biquad<float> s1{ c1 };

    std::array<filters::passive::BiquadCoeffs<float>, 2> cascadeCoeffs{ c0, c1 };
    filters::passive::BiquadCascade<float, 2> cascade{ cascadeCoeffs };

    constexpr float tol{ 1e-6f };
    for (int i = 0; i < 20; ++i)
    {
        const float x{ static_cast<float>(i) * 0.1f };
        const float expected{ s1.Filter(s0.Filter(x)) };
        EXPECT_NEAR(cascade.Filter(x), expected, tol);
    }
}

TEST_F(TestBiquadCascade, reset_clears_state)
{
    for (int i = 0; i < 50; ++i)
        filter.Filter(1.0f);

    filter.Reset();

    EXPECT_NEAR(filter.Filter(1.0f),
        filters::passive::Biquad<float>::LowPass(fc, fs, Q).b0 *
            filters::passive::Biquad<float>::LowPass(fc, fs, Q).b0,
        1e-5f);
}

TEST_F(TestBiquadCascade, bandpass_attenuates_dc)
{
    filters::passive::Biquad<float> bq{ filters::passive::Biquad<float>::BandPass(fc, fs, Q) };

    for (int i = 0; i < 500; ++i)
        bq.Filter(1.0f);

    EXPECT_NEAR(bq.Filter(1.0f), 0.0f, 1e-4f);
}

TEST_F(TestBiquadCascade, notch_rejects_center_freq)
{
    filters::passive::Biquad<float> bq{ filters::passive::Biquad<float>::Notch(fc, fs, Q) };
    const float w{ 2.0f * std::numbers::pi_v<float> * fc / fs };
    constexpr int settleSamples{ 500 };
    constexpr int measureSamples{ 100 };

    for (int i = 0; i < settleSamples; ++i)
        bq.Filter(std::sin(static_cast<float>(i) * w));

    float maxAmp{ 0.0f };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float y{ bq.Filter(std::sin(static_cast<float>(i) * w)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_LT(maxAmp, 0.01f);
}

TEST_F(TestBiquadCascade, notch_passes_dc)
{
    filters::passive::Biquad<float> bq{ filters::passive::Biquad<float>::Notch(fc, fs, Q) };

    for (int i = 0; i < 500; ++i)
        bq.Filter(1.0f);

    EXPECT_NEAR(bq.Filter(1.0f), 1.0f, 1e-3f);
}

TEST_F(TestBiquadCascade, peaking_boosts_center_freq)
{
    constexpr float gainDb{ 6.0f };
    filters::passive::Biquad<float> bq{ filters::passive::Biquad<float>::Peaking(fc, fs, Q, gainDb) };
    const float w{ 2.0f * std::numbers::pi_v<float> * fc / fs };
    constexpr int settleSamples{ 500 };
    constexpr int measureSamples{ 100 };

    for (int i = 0; i < settleSamples; ++i)
        bq.Filter(std::sin(static_cast<float>(i) * w));

    double sumSquares{ 0.0 };
    for (int i = settleSamples; i < settleSamples + measureSamples; ++i)
    {
        const float y{ bq.Filter(std::sin(static_cast<float>(i) * w)) };
        sumSquares += static_cast<double>(y) * y;
    }

    const float amplitude{ static_cast<float>(std::sqrt(2.0 * sumSquares / measureSamples)) };
    EXPECT_NEAR(amplitude, std::pow(10.0f, gainDb / 20.0f), math::Tolerance<float>());
}
