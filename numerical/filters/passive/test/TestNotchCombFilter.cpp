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

    constexpr float kFs{ 1000.0f };
    constexpr float kF0{ 50.0f };
    constexpr float kPi{ std::numbers::pi_v<float> };
}

TEST_F(TestNotchComb, notch_rejects_target_frequency)
{
    const float w0{ 2.0f * kPi * kF0 / kFs };

    for (int i{ 0 }; i < 500; ++i)
        notch.Filter(std::sin(static_cast<float>(i) * w0));

    float maxAmp{ 0.0f };
    for (int i{ 500 }; i < 600; ++i)
    {
        const float y{ notch.Filter(std::sin(static_cast<float>(i) * w0)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_NEAR(maxAmp, 0.0f, 0.01f);
}

TEST_F(TestNotchComb, notch_passes_off_frequency)
{
    const float w{ 2.0f * kPi * 250.0f / kFs };

    for (int i{ 0 }; i < 200; ++i)
        notch.Filter(std::sin(static_cast<float>(i) * w));

    float maxAmp{ 0.0f };
    for (int i{ 200 }; i < 300; ++i)
    {
        const float y{ notch.Filter(std::sin(static_cast<float>(i) * w)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_NEAR(maxAmp, 1.0f, 0.01f);
}

TEST_F(TestNotchComb, notch_dc_gain_is_unity)
{
    constexpr float dc{ 0.7f };

    float last{ 0.0f };
    for (int i{ 0 }; i < 300; ++i)
        last = notch.Filter(dc);

    EXPECT_NEAR(last, dc, math::Tolerance<float>());
}

TEST_F(TestNotchComb, notch_nyquist_gain_is_unity)
{
    float maxAmp{ 0.0f };
    for (int i{ 0 }; i < 300; ++i)
    {
        const float x{ (i % 2 == 0) ? 1.0f : -1.0f };
        const float y{ notch.Filter(x) };
        if (i >= 200)
        {
            const float absY{ y < 0.0f ? -y : y };
            if (absY > maxAmp)
                maxAmp = absY;
        }
    }

    EXPECT_NEAR(maxAmp, 1.0f, 0.01f);
}

TEST_F(TestNotchComb, notch_high_q_narrow_rejection_band)
{
    filters::passive::NotchFilter<float> narrowNotch{ 50.0f, 1000.0f, 10.0f };
    const float w45{ 2.0f * kPi * 45.0f / kFs };

    for (int i{ 0 }; i < 500; ++i)
        narrowNotch.Filter(std::sin(static_cast<float>(i) * w45));

    float maxAmp{ 0.0f };
    for (int i{ 500 }; i < 600; ++i)
    {
        const float y{ narrowNotch.Filter(std::sin(static_cast<float>(i) * w45)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_GT(maxAmp, 0.85f);
}

TEST_F(TestNotchComb, notch_low_q_wide_rejection_band)
{
    filters::passive::NotchFilter<float> wideNotch{ 50.0f, 1000.0f, 2.0f };
    const float w45{ 2.0f * kPi * 45.0f / kFs };

    for (int i{ 0 }; i < 500; ++i)
        wideNotch.Filter(std::sin(static_cast<float>(i) * w45));

    float maxAmp{ 0.0f };
    for (int i{ 500 }; i < 600; ++i)
    {
        const float y{ wideNotch.Filter(std::sin(static_cast<float>(i) * w45)) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_LT(maxAmp, 0.45f);
}

TEST_F(TestNotchComb, notch_reset_restores_initial_state)
{
    const float w0{ 2.0f * kPi * kF0 / kFs };

    for (int i{ 0 }; i < 50; ++i)
        notch.Filter(std::sin(static_cast<float>(i) * w0));

    notch.Reset();

    filters::passive::NotchFilter<float> fresh{ 50.0f, 1000.0f, 10.0f };

    std::array<float, 20> postReset{};
    std::array<float, 20> freshRun{};
    for (int i{ 0 }; i < 20; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w0 * 0.7f) };
        postReset[static_cast<std::size_t>(i)] = notch.Filter(x);
        freshRun[static_cast<std::size_t>(i)] = fresh.Filter(x);
    }

    for (int i{ 0 }; i < 20; ++i)
        EXPECT_NEAR(postReset[static_cast<std::size_t>(i)],
                    freshRun[static_cast<std::size_t>(i)],
                    1e-5f);
}

TEST_F(TestNotchComb, notch_zero_input_zero_output_after_reset)
{
    for (int i{ 0 }; i < 50; ++i)
        notch.Filter(1.0f);

    notch.Reset();

    EXPECT_NEAR(notch.Filter(0.0f), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(notch.Filter(0.0f), 0.0f, math::Tolerance<float>());
}

TEST_F(TestNotchComb, notch_two_instances_independent)
{
    filters::passive::NotchFilter<float> a{ 50.0f, 1000.0f, 10.0f };
    filters::passive::NotchFilter<float> b{ 50.0f, 1000.0f, 10.0f };

    const float w0{ 2.0f * kPi * kF0 / kFs };
    std::array<float, 30> ra{};
    std::array<float, 30> rb{};

    for (std::size_t i{ 0 }; i < 30; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w0) };
        ra[i] = a.Filter(x);
        rb[i] = b.Filter(x);
    }

    for (std::size_t i{ 0 }; i < 30; ++i)
        EXPECT_FLOAT_EQ(ra[i], rb[i]);
}

TEST_F(TestNotchComb, comb_feedforward_exact_impulse_response)
{
    const float y0{ comb.Filter(1.0f) };
    EXPECT_FLOAT_EQ(y0, 1.0f);

    for (int i{ 1 }; i < 20; ++i)
    {
        const float y{ comb.Filter(0.0f) };
        EXPECT_NEAR(y, 0.0f, math::Tolerance<float>());
    }

    const float y20{ comb.Filter(0.0f) };
    EXPECT_NEAR(y20, -0.9f, math::Tolerance<float>());
}

TEST_F(TestNotchComb, comb_feedforward_gain_zero_is_passthrough)
{
    filters::passive::CombFilter<float, 20, false> passthrough{ 0.0f };

    std::array<float, 25> inputs{};
    for (std::size_t i{ 0 }; i < 25; ++i)
        inputs[i] = std::sin(static_cast<float>(i) * 0.3f);

    for (std::size_t i{ 0 }; i < 25; ++i)
        EXPECT_FLOAT_EQ(passthrough.Filter(inputs[i]), inputs[i]);
}

TEST_F(TestNotchComb, comb_feedforward_rejects_harmonics_of_delay)
{
    const float w1{ 2.0f * kPi * 50.0f / kFs };
    const float w2{ 2.0f * kPi * 100.0f / kFs };
    const float w3{ 2.0f * kPi * 150.0f / kFs };

    for (int i{ 0 }; i < 200; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w1) +
                       std::sin(static_cast<float>(i) * w2) +
                       std::sin(static_cast<float>(i) * w3) };
        comb.Filter(x);
    }

    float maxAmp{ 0.0f };
    for (int i{ 200 }; i < 240; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * w1) +
                       std::sin(static_cast<float>(i) * w2) +
                       std::sin(static_cast<float>(i) * w3) };
        const float y{ comb.Filter(x) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > maxAmp)
            maxAmp = absY;
    }

    EXPECT_LT(maxAmp, 0.35f);
}

TEST_F(TestNotchComb, comb_feedforward_delay_wraparound_correct)
{
    std::array<float, 45> inputs{};
    for (std::size_t i{ 0 }; i < 45; ++i)
        inputs[i] = static_cast<float>(i + 1) * 0.1f;

    std::array<float, 45> outputs{};
    for (std::size_t i{ 0 }; i < 45; ++i)
        outputs[i] = comb.Filter(inputs[i]);

    for (std::size_t i{ 20 }; i < 45; ++i)
    {
        const float expected{ inputs[i] - 0.9f * inputs[i - 20] };
        EXPECT_NEAR(outputs[i], expected, math::Tolerance<float>());
    }
}

TEST_F(TestNotchComb, comb_reset_clears_delay_line)
{
    for (int i{ 0 }; i < 30; ++i)
        comb.Filter(1.0f);

    comb.Reset();

    filters::passive::CombFilter<float, 20, false> fresh{ 0.9f };

    std::array<float, 25> postReset{};
    std::array<float, 25> freshRun{};
    for (std::size_t i{ 0 }; i < 25; ++i)
    {
        const float x{ std::sin(static_cast<float>(i) * 0.3f) };
        postReset[i] = comb.Filter(x);
        freshRun[i] = fresh.Filter(x);
    }

    for (std::size_t i{ 0 }; i < 25; ++i)
        EXPECT_FLOAT_EQ(postReset[i], freshRun[i]);
}

TEST_F(TestNotchComb, feedback_comb_stable_bounded_output)
{
    filters::passive::CombFilter<float, 20, true> fbComb{ 0.95f };

    float peakAfterInitial{ 0.0f };
    fbComb.Filter(1.0f);

    for (int i{ 1 }; i < 400; ++i)
    {
        const float y{ fbComb.Filter(0.0f) };
        const float absY{ y < 0.0f ? -y : y };
        if (absY > peakAfterInitial)
            peakAfterInitial = absY;
    }

    EXPECT_LT(peakAfterInitial, 1.0f);
}

TEST_F(TestNotchComb, feedback_comb_reset_clears_state)
{
    filters::passive::CombFilter<float, 20, true> fbComb{ 0.95f };

    fbComb.Filter(1.0f);
    for (int i{ 0 }; i < 30; ++i)
        fbComb.Filter(0.0f);

    fbComb.Reset();

    EXPECT_NEAR(fbComb.Filter(0.0f), 0.0f, math::Tolerance<float>());
}
