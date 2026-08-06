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
    protected:
        static constexpr float kFs{ 1000.0f };
        static constexpr float kFc{ 100.0f };
        static constexpr float kQ{ 0.707f };

        static filters::passive::BiquadCoeffs<float> LpCoeffs()
        {
            return filters::passive::Biquad<float>::LowPass(kFc, kFs, kQ);
        }

        static filters::passive::BiquadCoeffs<float> HpCoeffs()
        {
            return filters::passive::Biquad<float>::HighPass(kFc, kFs, kQ);
        }

        static filters::passive::BiquadCoeffs<float> BpCoeffs()
        {
            return filters::passive::Biquad<float>::BandPass(kFc, kFs, kQ);
        }

        static filters::passive::BiquadCoeffs<float> NotchCoeffs()
        {
            return filters::passive::Biquad<float>::Notch(kFc, kFs, kQ);
        }

        static float RunSteadyStateRms(filters::passive::Biquad<float>& bq, float freq, int settle, int measure)
        {
            const float w{ 2.0f * std::numbers::pi_v<float> * freq / kFs };
            for (int i = 0; i < settle; ++i)
                bq.Filter(std::sin(static_cast<float>(i) * w));
            float sumSq{ 0.0f };
            for (int i = settle; i < settle + measure; ++i)
            {
                const float y{ bq.Filter(std::sin(static_cast<float>(i) * w)) };
                sumSq += y * y;
            }
            return std::sqrt(2.0f * sumSq / static_cast<float>(measure));
        }
    };
}

TEST_F(TestBiquadCascade, lowpass_coefficients_match_closed_form)
{
    const auto c{ LpCoeffs() };
    EXPECT_NEAR(c.b0, 0.06745228f, 1e-6f);
    EXPECT_NEAR(c.b1, 0.13490457f, 1e-6f);
    EXPECT_NEAR(c.b2, 0.06745228f, 1e-6f);
    EXPECT_NEAR(c.a1, -1.14292982f, 1e-6f);
    EXPECT_NEAR(c.a2, 0.41273895f, 1e-6f);
}

TEST_F(TestBiquadCascade, highpass_coefficients_match_closed_form)
{
    const auto c{ HpCoeffs() };
    EXPECT_NEAR(c.b0, 0.63891719f, 1e-5f);
    EXPECT_NEAR(c.b1, -1.27783439f, 1e-5f);
    EXPECT_NEAR(c.b2, 0.63891719f, 1e-5f);
    EXPECT_NEAR(c.a1, -1.14292982f, 1e-5f);
    EXPECT_NEAR(c.a2, 0.41273895f, 1e-5f);
}

TEST_F(TestBiquadCascade, lowpass_dc_gain_is_unity)
{
    filters::passive::Biquad<float> bq{ LpCoeffs() };
    const auto c{ LpCoeffs() };
    const float dcGain{ (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2) };
    EXPECT_NEAR(dcGain, 1.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, lowpass_nyquist_gain_is_zero)
{
    const auto c{ LpCoeffs() };
    const float nyGain{ (c.b0 - c.b1 + c.b2) / (1.0f - c.a1 + c.a2) };
    EXPECT_NEAR(nyGain, 0.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, highpass_dc_gain_is_zero)
{
    const auto c{ HpCoeffs() };
    const float dcGain{ (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2) };
    EXPECT_NEAR(dcGain, 0.0f, 1e-6f);
}

TEST_F(TestBiquadCascade, highpass_nyquist_gain_is_unity)
{
    const auto c{ HpCoeffs() };
    const float nyGain{ (c.b0 - c.b1 + c.b2) / (1.0f - c.a1 + c.a2) };
    EXPECT_NEAR(nyGain, 1.0f, 1e-5f);
}

TEST_F(TestBiquadCascade, bandpass_dc_gain_is_zero)
{
    const auto c{ BpCoeffs() };
    const float dcGain{ (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2) };
    EXPECT_NEAR(dcGain, 0.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, notch_dc_gain_is_unity)
{
    const auto c{ NotchCoeffs() };
    const float dcGain{ (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2) };
    EXPECT_NEAR(dcGain, 1.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, notch_nyquist_gain_is_unity)
{
    const auto c{ NotchCoeffs() };
    const float nyGain{ (c.b0 - c.b1 + c.b2) / (1.0f - c.a1 + c.a2) };
    EXPECT_NEAR(nyGain, 1.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, peaking_dc_gain_is_unity)
{
    const auto c{ filters::passive::Biquad<float>::Peaking(kFc, kFs, kQ, 6.0f) };
    const float dcGain{ (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2) };
    EXPECT_NEAR(dcGain, 1.0f, 1e-5f);
}

TEST_F(TestBiquadCascade, peaking_nyquist_gain_is_unity)
{
    const auto c{ filters::passive::Biquad<float>::Peaking(kFc, kFs, kQ, 6.0f) };
    const float nyGain{ (c.b0 - c.b1 + c.b2) / (1.0f - c.a1 + c.a2) };
    EXPECT_NEAR(nyGain, 1.0f, 1e-5f);
}

TEST_F(TestBiquadCascade, lowpass_pole_radius_less_than_one)
{
    const auto c{ LpCoeffs() };
    const float poleRadius{ std::sqrt(c.a2) };
    EXPECT_LT(poleRadius, 1.0f);
    EXPECT_NEAR(poleRadius, 0.64244763f, 1e-5f);
}

TEST_F(TestBiquadCascade, lowpass_impulse_decays_to_zero)
{
    filters::passive::Biquad<float> bq{ LpCoeffs() };
    bq.Filter(1.0f);
    for (int i = 0; i < 199; ++i)
        bq.Filter(0.0f);
    const float last{ bq.Filter(0.0f) };
    EXPECT_NEAR(last, 0.0f, 1e-20f);
}

TEST_F(TestBiquadCascade, biquad_impulse_response_matches_hand_computation)
{
    const filters::passive::BiquadCoeffs<float> c{ 0.5f, 0.25f, 0.1f, -0.5f, 0.1f };
    filters::passive::Biquad<float> bq{ c };

    constexpr std::array<float, 5> expected{ 0.5f, 0.5f, 0.3f, 0.1f, 0.02f };

    EXPECT_NEAR(bq.Filter(1.0f), expected[0], 1e-5f);
    EXPECT_NEAR(bq.Filter(0.0f), expected[1], 1e-5f);
    EXPECT_NEAR(bq.Filter(0.0f), expected[2], 1e-5f);
    EXPECT_NEAR(bq.Filter(0.0f), expected[3], 1e-5f);
    EXPECT_NEAR(bq.Filter(0.0f), expected[4], 1e-5f);
}

TEST_F(TestBiquadCascade, bypass_coeffs_are_passthrough)
{
    const filters::passive::BiquadCoeffs<float> bypass{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    filters::passive::Biquad<float> bq{ bypass };

    EXPECT_NEAR(bq.Filter(1.0f), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(bq.Filter(0.5f), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(bq.Filter(-2.0f), -2.0f, math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, zero_input_produces_zero_output)
{
    filters::passive::Biquad<float> bq{ LpCoeffs() };
    for (int i = 0; i < 100; ++i)
        EXPECT_FLOAT_EQ(bq.Filter(0.0f), 0.0f);
}

TEST_F(TestBiquadCascade, cascade_dc_gain_is_unity)
{
    std::array<filters::passive::BiquadCoeffs<float>, 2> coeffs{ LpCoeffs(), LpCoeffs() };
    filters::passive::BiquadCascade<float, 2> cascade{ coeffs };

    constexpr int kSettle{ 2000 };
    float last{ 0.0f };
    for (int i = 0; i < kSettle; ++i)
        last = cascade.Filter(1.0f);

    EXPECT_NEAR(last, 1.0f, 1e-3f);
}

TEST_F(TestBiquadCascade, cascade_nyquist_attenuation)
{
    std::array<filters::passive::BiquadCoeffs<float>, 2> coeffs{ LpCoeffs(), LpCoeffs() };
    filters::passive::BiquadCascade<float, 2> cascade{ coeffs };

    const float w{ 2.0f * std::numbers::pi_v<float> * 490.0f / kFs };
    for (int i = 0; i < 500; ++i)
        cascade.Filter(std::sin(static_cast<float>(i) * w));

    float maxAmp{ 0.0f };
    for (int i = 500; i < 600; ++i)
    {
        const float y{ cascade.Filter(std::sin(static_cast<float>(i) * w)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_LT(maxAmp, 0.01f);
}

TEST_F(TestBiquadCascade, cascade_equals_serial_sections)
{
    const auto c0{ LpCoeffs() };
    const auto c1{ filters::passive::Biquad<float>::HighPass(200.0f, kFs, kQ) };

    filters::passive::Biquad<float> s0{ c0 };
    filters::passive::Biquad<float> s1{ c1 };

    std::array<filters::passive::BiquadCoeffs<float>, 2> cascadeCoeffs{ c0, c1 };
    filters::passive::BiquadCascade<float, 2> cascade{ cascadeCoeffs };

    for (int i = 0; i < 20; ++i)
    {
        const float x{ static_cast<float>(i) * 0.1f };
        EXPECT_FLOAT_EQ(cascade.Filter(x), s1.Filter(s0.Filter(x)));
    }
}

TEST_F(TestBiquadCascade, reset_clears_state_to_zero_initial)
{
    std::array<filters::passive::BiquadCoeffs<float>, 2> coeffs{ LpCoeffs(), LpCoeffs() };
    filters::passive::BiquadCascade<float, 2> cascade{ coeffs };

    for (int i = 0; i < 50; ++i)
        cascade.Filter(1.0f);

    cascade.Reset();

    filters::passive::BiquadCascade<float, 2> fresh{ coeffs };
    EXPECT_FLOAT_EQ(cascade.Filter(1.0f), fresh.Filter(1.0f));
}

TEST_F(TestBiquadCascade, determinism_same_input_gives_identical_output)
{
    std::array<filters::passive::BiquadCoeffs<float>, 2> coeffs{ LpCoeffs(), LpCoeffs() };
    filters::passive::BiquadCascade<float, 2> cascade1{ coeffs };
    filters::passive::BiquadCascade<float, 2> cascade2{ coeffs };

    for (int i = 0; i < 100; ++i)
    {
        const float x{ static_cast<float>(i) * 0.05f };
        EXPECT_FLOAT_EQ(cascade1.Filter(x), cascade2.Filter(x));
    }
}

TEST_F(TestBiquadCascade, two_instances_do_not_interfere)
{
    std::array<filters::passive::BiquadCoeffs<float>, 2> coeffs{ LpCoeffs(), LpCoeffs() };
    filters::passive::BiquadCascade<float, 2> cascadeA{ coeffs };
    filters::passive::BiquadCascade<float, 2> cascadeRef{ coeffs };

    for (int i = 0; i < 50; ++i)
        cascadeA.Filter(static_cast<float>(i));

    for (int i = 0; i < 20; ++i)
    {
        const float x{ static_cast<float>(i) * 0.1f };
        filters::passive::BiquadCascade<float, 2> solo{ coeffs };
        float soloY{ 0.0f };
        for (int k = 0; k <= i; ++k)
            soloY = solo.Filter(static_cast<float>(k) * 0.1f);
        const float refY{ cascadeRef.Filter(x) };
        EXPECT_FLOAT_EQ(refY, soloY);
    }
}

TEST_F(TestBiquadCascade, biquad_reset_matches_fresh_instance)
{
    filters::passive::Biquad<float> bq{ LpCoeffs() };
    for (int i = 0; i < 30; ++i)
        bq.Filter(1.0f);
    bq.Reset();

    filters::passive::Biquad<float> fresh{ LpCoeffs() };
    EXPECT_FLOAT_EQ(bq.Filter(1.0f), fresh.Filter(1.0f));
    EXPECT_FLOAT_EQ(bq.Filter(0.5f), fresh.Filter(0.5f));
}

TEST_F(TestBiquadCascade, linearity_scaling_homogeneity)
{
    constexpr float kScale{ 3.14159f };
    const filters::passive::BiquadCoeffs<float> c{ 0.5f, 0.25f, 0.1f, -0.5f, 0.1f };

    constexpr std::array<float, 8> inputs{ 1.0f, -0.5f, 0.3f, 0.7f, -0.2f, 0.9f, -0.4f, 0.1f };

    for (std::size_t i = 0; i < inputs.size(); ++i)
    {
        filters::passive::Biquad<float> bqUnit{ c };
        filters::passive::Biquad<float> bqScaled{ c };

        float yUnit{ 0.0f };
        float yScaled{ 0.0f };
        for (std::size_t k = 0; k <= i; ++k)
        {
            yUnit = bqUnit.Filter(inputs[k]);
            yScaled = bqScaled.Filter(kScale * inputs[k]);
        }
        EXPECT_NEAR(yScaled, kScale * yUnit, 1e-5f);
    }
}

TEST_F(TestBiquadCascade, linearity_superposition)
{
    const filters::passive::BiquadCoeffs<float> c{ 0.5f, 0.25f, 0.1f, -0.5f, 0.1f };
    constexpr std::array<float, 6> xA{ 1.0f, 0.5f, -0.3f, 0.8f, -0.1f, 0.6f };
    constexpr std::array<float, 6> xB{ -0.5f, 0.3f, 0.7f, -0.4f, 0.9f, -0.2f };

    filters::passive::Biquad<float> bqA{ c };
    filters::passive::Biquad<float> bqB{ c };
    filters::passive::Biquad<float> bqSum{ c };

    for (std::size_t i = 0; i < xA.size(); ++i)
    {
        const float yA{ bqA.Filter(xA[i]) };
        const float yB{ bqB.Filter(xB[i]) };
        const float ySum{ bqSum.Filter(xA[i] + xB[i]) };
        EXPECT_NEAR(ySum, yA + yB, 1e-5f);
    }
}

TEST_F(TestBiquadCascade, bandpass_peak_gain_equals_q)
{
    filters::passive::Biquad<float> bq{ BpCoeffs() };
    const float amplitude{ RunSteadyStateRms(bq, kFc, 1000, 200) };
    EXPECT_NEAR(amplitude, kQ, 1e-2f);
}

TEST_F(TestBiquadCascade, notch_rejects_center_freq)
{
    filters::passive::Biquad<float> bq{ NotchCoeffs() };
    const float amplitude{ RunSteadyStateRms(bq, kFc, 500, 100) };
    EXPECT_LT(amplitude, 0.01f);
}

TEST_F(TestBiquadCascade, peaking_boosts_center_freq)
{
    constexpr float kGainDb{ 6.0f };
    filters::passive::Biquad<float> bq{ filters::passive::Biquad<float>::Peaking(kFc, kFs, kQ, kGainDb) };
    const float amplitude{ RunSteadyStateRms(bq, kFc, 500, 100) };
    EXPECT_NEAR(amplitude, std::pow(10.0f, kGainDb / 20.0f), math::Tolerance<float>());
}

TEST_F(TestBiquadCascade, lowpass_bounded_output_long_run)
{
    filters::passive::Biquad<float> bq{ LpCoeffs() };
    const float w{ 2.0f * std::numbers::pi_v<float> * 50.0f / kFs };
    bool anyNan{ false };
    bool anyInf{ false };
    for (int i = 0; i < 10000; ++i)
    {
        const float y{ bq.Filter(std::sin(static_cast<float>(i) * w)) };
        if (std::isnan(y))
            anyNan = true;
        if (std::isinf(y))
            anyInf = true;
    }
    EXPECT_FALSE(anyNan);
    EXPECT_FALSE(anyInf);
}
