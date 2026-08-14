#include "numerical/control_analysis/FrequencyResponse.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <tuple>

namespace
{
    static constexpr float kSampleFrequency = 1000.0f;

    class TestFrequencyResponseUnity : public ::testing::Test
    {
    protected:
        std::array<float, 1> b{ 1.0f };
        std::array<float, 1> a{ 1.0f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };

    class TestFrequencyResponseLowpass : public ::testing::Test
    {
    protected:
        std::array<float, 2> b{ 0.5f, 0.5f };
        std::array<float, 1> a{ 1.0f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };

    class TestFrequencyResponseFirstOrderIir : public ::testing::Test
    {
    protected:
        std::array<float, 1> b{ 1.0f };
        std::array<float, 2> a{ 1.0f, -0.5f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };

    class TestFrequencyResponsePureDelay : public ::testing::Test
    {
    protected:
        std::array<float, 2> b{ 0.0f, 1.0f };
        std::array<float, 1> a{ 1.0f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };

    class TestFrequencyResponseBiquad : public ::testing::Test
    {
    protected:
        std::array<float, 3> b{ 0.25f, 0.5f, 0.25f };
        std::array<float, 3> a{ 1.0f, -0.5f, 0.1f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };

    class TestFrequencyResponseHighpass : public ::testing::Test
    {
    protected:
        std::array<float, 2> b{ 0.5f, -0.5f };
        std::array<float, 1> a{ 1.0f };
        control_analysis::FrequencyResponse<float, 64> freqResponse{ b, a, kSampleFrequency };
    };
}

TEST_F(TestFrequencyResponseUnity, unity_system_has_zero_db_response)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(frequencies.empty());
    ASSERT_EQ(frequencies.size(), magnitudes.size());
    ASSERT_EQ(frequencies.size(), phases.size());

    for (std::size_t i = 0; i < magnitudes.size(); ++i)
        EXPECT_NEAR(magnitudes[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestFrequencyResponseUnity, unity_system_has_zero_phase)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (std::size_t i = 0; i < phases.size(); ++i)
        EXPECT_NEAR(phases[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestFrequencyResponseUnity, output_vector_sizes_equal_number_of_points)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    EXPECT_EQ(frequencies.size(), 64u);
    EXPECT_EQ(magnitudes.size(), 64u);
    EXPECT_EQ(phases.size(), 64u);
}

TEST_F(TestFrequencyResponseUnity, frequencies_span_up_to_nyquist)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(frequencies.empty());
    EXPECT_NEAR(frequencies.back(), kSampleFrequency / 2.0f, 1.0f);
}

TEST_F(TestFrequencyResponseUnity, frequencies_are_positive_and_bounded)
{
    std::array<float, 1> b128{ 1.0f };
    std::array<float, 1> a128{ 1.0f };
    control_analysis::FrequencyResponse<float, 128> fr128{ b128, a128, kSampleFrequency };

    auto [frequencies, magnitudes, phases] = fr128.Calculate();

    ASSERT_FALSE(frequencies.empty());
    for (const auto& f : frequencies)
    {
        EXPECT_GT(f, 0.0f);
        EXPECT_LE(f, kSampleFrequency / 2.0f);
    }
}

TEST_F(TestFrequencyResponseUnity, determinism_same_input_produces_identical_output)
{
    auto [freq1, mag1, phase1] = freqResponse.Calculate();

    std::array<float, 1> b2{ 1.0f };
    std::array<float, 1> a2{ 1.0f };
    control_analysis::FrequencyResponse<float, 64> fr2{ b2, a2, kSampleFrequency };
    auto [freq2, mag2, phase2] = fr2.Calculate();

    ASSERT_EQ(mag1.size(), mag2.size());
    for (std::size_t i = 0; i < mag1.size(); ++i)
    {
        EXPECT_FLOAT_EQ(mag1[i], mag2[i]);
        EXPECT_FLOAT_EQ(phase1[i], phase2[i]);
        EXPECT_FLOAT_EQ(freq1[i], freq2[i]);
    }
}

TEST_F(TestFrequencyResponseLowpass, frequencies_are_monotonically_increasing)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_GT(frequencies.size(), 1u);
    for (std::size_t i = 1; i < frequencies.size(); ++i)
        EXPECT_GT(frequencies[i], frequencies[i - 1]);
}

TEST_F(TestFrequencyResponseLowpass, lowpass_attenuates_high_frequencies_relative_to_low)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_GT(magnitudes.size(), 2u);
    EXPECT_LT(magnitudes.back(), magnitudes.front());
}

TEST_F(TestFrequencyResponseLowpass, phase_response_is_within_valid_range)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (const auto& p : phases)
    {
        EXPECT_GE(p, -180.0f);
        EXPECT_LE(p, 180.0f);
    }
}

TEST_F(TestFrequencyResponseLowpass, magnitude_at_quarter_nyquist_is_minus_3db)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    float bestMag = 0.0f;
    float bestPhase = 0.0f;
    float targetFreq = kSampleFrequency / 4.0f;
    float minDiff = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < frequencies.size(); ++i)
    {
        float diff = std::abs(frequencies[i] - targetFreq);
        if (diff < minDiff)
        {
            minDiff = diff;
            bestMag = magnitudes[i];
            bestPhase = phases[i];
        }
    }

    EXPECT_NEAR(bestMag, -3.0103f, 0.5f);
    EXPECT_NEAR(bestPhase, -45.0f, 2.0f);
}

TEST_F(TestFrequencyResponseLowpass, magnitude_is_nonpositive_across_all_frequencies)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (const auto& m : magnitudes)
        EXPECT_LE(m, 0.1f);
}

TEST_F(TestFrequencyResponseFirstOrderIir, dc_gain_matches_analytic_value)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(frequencies.empty());
    EXPECT_NEAR(magnitudes.front(), 20.0f * std::log10(2.0f), 0.5f);
}

TEST_F(TestFrequencyResponseFirstOrderIir, high_frequency_gain_matches_analytic_value)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(magnitudes.empty());
    float expectedDb = 20.0f * std::log10(1.0f / 1.5f);
    EXPECT_NEAR(magnitudes.back(), expectedDb, 0.5f);
}

TEST_F(TestFrequencyResponseFirstOrderIir, output_sizes_match_number_of_points)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    EXPECT_EQ(frequencies.size(), 64u);
    EXPECT_EQ(magnitudes.size(), 64u);
    EXPECT_EQ(phases.size(), 64u);
}

TEST_F(TestFrequencyResponsePureDelay, magnitude_is_zero_db_at_all_frequencies)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (const auto& m : magnitudes)
        EXPECT_NEAR(m, 0.0f, math::Tolerance<float>());
}

TEST_F(TestFrequencyResponsePureDelay, phase_is_negative_below_quarter_nyquist)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(phases.empty());
    for (std::size_t i = 0; i < phases.size(); ++i)
    {
        if (frequencies[i] < kSampleFrequency / 4.0f)
            EXPECT_LT(phases[i], 0.0f);
    }
}

TEST_F(TestFrequencyResponsePureDelay, phase_at_sample_frequency_over_four_is_minus_90_degrees)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    float targetFreq = kSampleFrequency / 4.0f;
    float bestPhase = 0.0f;
    float minDiff = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < frequencies.size(); ++i)
    {
        float diff = std::abs(frequencies[i] - targetFreq);
        if (diff < minDiff)
        {
            minDiff = diff;
            bestPhase = phases[i];
        }
    }

    EXPECT_NEAR(bestPhase, -90.0f, 5.0f);
}

TEST_F(TestFrequencyResponseBiquad, output_sizes_match)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    EXPECT_EQ(frequencies.size(), magnitudes.size());
    EXPECT_EQ(frequencies.size(), phases.size());
    EXPECT_EQ(frequencies.size(), 64u);
}

TEST_F(TestFrequencyResponseBiquad, magnitude_has_no_nan_or_inf)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (const auto& m : magnitudes)
    {
        EXPECT_FALSE(std::isnan(m));
        EXPECT_FALSE(std::isinf(m));
    }
}

TEST_F(TestFrequencyResponseBiquad, phase_has_no_nan_or_inf)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    for (const auto& p : phases)
    {
        EXPECT_FALSE(std::isnan(p));
        EXPECT_FALSE(std::isinf(p));
    }
}

TEST_F(TestFrequencyResponseHighpass, attenuates_low_frequencies_relative_to_high)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_GT(magnitudes.size(), 2u);
    EXPECT_LT(magnitudes.front(), magnitudes.back());
}

TEST_F(TestFrequencyResponseHighpass, magnitude_at_quarter_nyquist_is_minus_3db)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    float targetFreq = kSampleFrequency / 4.0f;
    float bestMag = 0.0f;
    float minDiff = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < frequencies.size(); ++i)
    {
        float diff = std::abs(frequencies[i] - targetFreq);
        if (diff < minDiff)
        {
            minDiff = diff;
            bestMag = magnitudes[i];
        }
    }

    EXPECT_NEAR(bestMag, -3.0103f, 0.5f);
}

TEST_F(TestFrequencyResponseUnity, zero_denominator_coefficients_produce_finite_output)
{
    std::array<float, 1> bz{ 1.0f };
    std::array<float, 1> az{ 0.0f };
    control_analysis::FrequencyResponse<float, 64> frZeroDenom{ bz, az, kSampleFrequency };

    auto [frequencies, magnitudes, phases] = frZeroDenom.Calculate();

    for (const auto& m : magnitudes)
    {
        EXPECT_FALSE(std::isnan(m));
        EXPECT_FALSE(std::isinf(m));
    }
}

TEST_F(TestFrequencyResponseUnity, magnitude_output_size_matches_points_for_128_points)
{
    std::array<float, 1> b{ 1.0f };
    std::array<float, 1> a{ 1.0f };
    control_analysis::FrequencyResponse<float, 128> fr128{ b, a, kSampleFrequency };

    auto [frequencies, magnitudes, phases] = fr128.Calculate();

    EXPECT_EQ(magnitudes.size(), 128u);
    EXPECT_EQ(phases.size(), 128u);
    EXPECT_EQ(frequencies.size(), 128u);
}

TEST_F(TestFrequencyResponseHighpass, phase_is_positive_at_low_frequencies)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(phases.empty());
    EXPECT_GT(phases.front(), 0.0f);
}

TEST_F(TestFrequencyResponseBiquad, dc_gain_is_finite_and_nonnan)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(magnitudes.empty());
    EXPECT_FALSE(std::isnan(magnitudes.front()));
    EXPECT_FALSE(std::isinf(magnitudes.front()));
}

TEST_F(TestFrequencyResponsePureDelay, phase_at_nyquist_is_minus_180_degrees)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(phases.empty());
    EXPECT_NEAR(std::abs(phases.back()), 180.0f, 5.0f);
}

TEST_F(TestFrequencyResponseFirstOrderIir, phase_is_negative_at_quarter_nyquist)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    float targetFreq = kSampleFrequency / 4.0f;
    float bestPhase = 0.0f;
    float minDiff = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < frequencies.size(); ++i)
    {
        float diff = std::abs(frequencies[i] - targetFreq);
        if (diff < minDiff)
        {
            minDiff = diff;
            bestPhase = phases[i];
        }
    }

    EXPECT_NEAR(bestPhase, -26.57f, 2.0f);
}

TEST_F(TestFrequencyResponseLowpass, lowpass_dc_gain_is_zero_db)
{
    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(magnitudes.empty());
    EXPECT_NEAR(magnitudes.front(), 0.0f, 0.1f);
}
