#include "numerical/control_analysis/FrequencyResponse.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestFrequencyResponse : public ::testing::Test
    {
    protected:
        static constexpr float kEpsilon = 1e-2f;
        static constexpr float sampleFrequency = 1000.0f;
    };
}

TEST_F(TestFrequencyResponse, unity_system_has_zero_db_response)
{
    std::array<float, 1> b = { 1.0f };
    std::array<float, 1> a = { 1.0f };

    control_analysis::FrequencyResponse<float, 64> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(frequencies.empty());
    ASSERT_EQ(frequencies.size(), magnitudes.size());
    ASSERT_EQ(frequencies.size(), phases.size());

    for (std::size_t i = 0; i < magnitudes.size(); ++i)
        EXPECT_NEAR(magnitudes[i], 0.0f, kEpsilon);
}

TEST_F(TestFrequencyResponse, frequencies_are_within_nyquist_range)
{
    std::array<float, 1> b = { 1.0f };
    std::array<float, 1> a = { 1.0f };

    control_analysis::FrequencyResponse<float, 128> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_FALSE(frequencies.empty());

    for (const auto& f : frequencies)
    {
        EXPECT_GT(f, 0.0f);
        EXPECT_LE(f, sampleFrequency / 2.0f);
    }
}

TEST_F(TestFrequencyResponse, frequencies_are_monotonically_increasing)
{
    std::array<float, 2> b = { 0.5f, 0.5f };
    std::array<float, 1> a = { 1.0f };

    control_analysis::FrequencyResponse<float, 64> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_GT(frequencies.size(), 1u);

    for (std::size_t i = 1; i < frequencies.size(); ++i)
        EXPECT_GT(frequencies[i], frequencies[i - 1]);
}

TEST_F(TestFrequencyResponse, lowpass_filter_attenuates_high_frequencies)
{
    std::array<float, 2> b = { 0.5f, 0.5f };
    std::array<float, 1> a = { 1.0f };

    control_analysis::FrequencyResponse<float, 64> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_GT(magnitudes.size(), 2u);

    auto lastMagnitude = magnitudes[magnitudes.size() - 1];
    auto firstMagnitude = magnitudes[0];

    EXPECT_LT(lastMagnitude, firstMagnitude);
}

TEST_F(TestFrequencyResponse, phase_response_is_computed)
{
    std::array<float, 2> b = { 0.5f, 0.5f };
    std::array<float, 1> a = { 1.0f };

    control_analysis::FrequencyResponse<float, 64> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    ASSERT_EQ(phases.size(), frequencies.size());

    for (const auto& p : phases)
    {
        EXPECT_GE(p, -180.0f);
        EXPECT_LE(p, 180.0f);
    }
}

TEST_F(TestFrequencyResponse, output_sizes_match_for_multi_coefficient_system)
{
    std::array<float, 3> b = { 0.25f, 0.5f, 0.25f };
    std::array<float, 3> a = { 1.0f, -0.5f, 0.1f };

    control_analysis::FrequencyResponse<float, 64> freqResponse(
        b, a, sampleFrequency);

    auto [frequencies, magnitudes, phases] = freqResponse.Calculate();

    EXPECT_EQ(frequencies.size(), magnitudes.size());
    EXPECT_EQ(frequencies.size(), phases.size());
    EXPECT_GT(frequencies.size(), 0u);
}
