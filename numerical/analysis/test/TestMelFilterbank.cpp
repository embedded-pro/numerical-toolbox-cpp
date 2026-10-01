#include "numerical/analysis/MelFilterbank.hpp"
#include <array>
#include <gtest/gtest.h>

namespace
{
    constexpr std::size_t fftSize{ 256 };
    constexpr std::size_t numMelBands{ 20 };
    constexpr float sampleRate{ 16000.0f };
    constexpr float fMin{ 20.0f };
    constexpr float fMax{ 8000.0f };

    using Filterbank = analysis::MelFilterbank<float, fftSize, numMelBands>;

    float RowSum(const Filterbank& filterbank, std::size_t band)
    {
        float sum{ 0.0f };
        for (std::size_t k = 0; k < Filterbank::NumBins; ++k)
            sum += filterbank.Weight(band, k);
        return sum;
    }

    class TestMelFilterbank
        : public ::testing::Test
    {};
}

TEST_F(TestMelFilterbank, hz_mel_conversion_round_trips_on_both_scales)
{
    EXPECT_NEAR(analysis::HzToMel(1000.0f, analysis::MelScale::Htk), 999.985537f, 1e-3f);
    EXPECT_NEAR(analysis::HzToMel(1000.0f, analysis::MelScale::Slaney), 15.0f, 1e-5f);
    EXPECT_NEAR(analysis::HzToMel(4000.0f, analysis::MelScale::Slaney), 35.163760f, 1e-4f);

    for (auto scale : { analysis::MelScale::Htk, analysis::MelScale::Slaney })
        for (float hz : { 20.0f, 440.0f, 999.0f, 1000.0f, 1001.0f, 4000.0f, 8000.0f })
            EXPECT_NEAR(analysis::MelToHz(analysis::HzToMel(hz, scale), scale), hz, hz * 1e-5f);
}

TEST_F(TestMelFilterbank, unnormalised_triangles_form_partition_of_unity)
{
    Filterbank filterbank{ sampleRate, fMin, fMax, analysis::MelScale::Slaney, analysis::MelNormalization::None };

    const float melLow{ analysis::HzToMel(fMin, analysis::MelScale::Slaney) };
    const float melStep{ (analysis::HzToMel(fMax, analysis::MelScale::Slaney) - melLow) / static_cast<float>(numMelBands + 1) };
    const float firstCentre{ analysis::MelToHz(melLow + melStep, analysis::MelScale::Slaney) };
    const float lastCentre{ analysis::MelToHz(melLow + static_cast<float>(numMelBands) * melStep, analysis::MelScale::Slaney) };

    for (std::size_t k = 0; k < Filterbank::NumBins; ++k)
    {
        const float hz{ static_cast<float>(k) * sampleRate / static_cast<float>(fftSize) };
        if (hz < firstCentre || hz > lastCentre)
            continue;

        float sum{ 0.0f };
        for (std::size_t m = 0; m < numMelBands; ++m)
            sum += filterbank.Weight(m, k);
        EXPECT_NEAR(sum, 1.0f, 1e-5f) << "bin " << k;
    }
}

TEST_F(TestMelFilterbank, slaney_filterbank_matches_librosa)
{
    constexpr std::array<float, numMelBands> librosaRowSums{ 0.015754f, 0.016273f, 0.016104f, 0.015754f, 0.015809f, 0.016573f, 0.015560f, 0.016141f, 0.016096f, 0.015918f, 0.015921f, 0.016126f, 0.015941f, 0.015981f, 0.016042f, 0.015967f, 0.016025f, 0.015991f, 0.016000f, 0.015992f };

    Filterbank filterbank{ sampleRate, fMin, fMax, analysis::MelScale::Slaney, analysis::MelNormalization::Slaney };

    for (std::size_t m = 0; m < numMelBands; ++m)
        EXPECT_NEAR(RowSum(filterbank, m), librosaRowSums[m], 1e-5f) << "band " << m;
}

TEST_F(TestMelFilterbank, htk_filterbank_matches_librosa)
{
    constexpr std::array<float, numMelBands> librosaRowSums{ 1.576777f, 1.702185f, 1.978733f, 2.197124f, 2.472485f, 2.793974f, 3.157482f, 3.526380f, 3.994063f, 4.480222f, 5.060342f, 5.687882f, 6.410446f, 7.202555f, 8.140608f, 9.142412f, 10.296803f, 11.599697f, 13.057395f, 14.695650f };

    Filterbank filterbank{ sampleRate, fMin, fMax, analysis::MelScale::Htk, analysis::MelNormalization::None };

    for (std::size_t m = 0; m < numMelBands; ++m)
        EXPECT_NEAR(RowSum(filterbank, m), librosaRowSums[m], 1e-4f) << "band " << m;
}
