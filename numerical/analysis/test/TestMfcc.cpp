#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/analysis/MelFilterbank.hpp"
#include "numerical/analysis/Mfcc.hpp"
#include "numerical/analysis/RealFastFourierTransform.hpp"
#include "numerical/analysis/TwiddleFactorsTable.hpp"
#include "numerical/analysis/windowing/Windowing.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestMfcc
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t fftSize{ 256 };
        static constexpr std::size_t numMelBands{ 20 };
        static constexpr std::size_t numCoefficients{ 13 };
        static constexpr float sampleRate{ 16000.0f };
        static constexpr float logFloor{ 1e-10f };

        analysis::TwiddleFactorsTable<float, fftSize / 4> engineTwiddles;
        analysis::TwiddleFactorsTable<float, fftSize / 2> realTwiddles;
        analysis::FastFourierTransformRadix2Impl<float, fftSize / 2> engine{ engineTwiddles };
        analysis::RealFastFourierTransform<float, fftSize> rfft{ engine, realTwiddles };
        windowing::HanningWindow<float> window;
        analysis::MelFilterbank<float, fftSize, numMelBands> filterbank{ sampleRate, 20.0f, 8000.0f, analysis::MelScale::Slaney, analysis::MelNormalization::Slaney };
        analysis::Mfcc<float, fftSize, numMelBands, numCoefficients> mfcc{ window, rfft, filterbank, logFloor };
        infra::BoundedVector<float>::WithMaxSize<fftSize> frame;
    };
}

TEST_F(TestMfcc, frame_matches_librosa_reference)
{
    constexpr std::array<float, numMelBands> referenceLogMel{ -1.231177f, 0.302156f, 2.162636f, -0.316564f, -7.896909f, -7.155183f, -6.513508f, -5.861534f, -5.282149f, -0.888027f, 0.569918f, -3.225003f, -3.274388f, -2.905951f, -2.630738f, -2.502352f, -2.568910f, -2.955704f, -3.865263f, -5.757557f };
    constexpr std::array<float, numCoefficients> referenceMfcc{ -13.818052f, 1.487473f, 2.082798f, 7.491946f, 3.916787f, 0.703016f, -5.453200f, -2.247771f, -1.511625f, -2.173861f, -3.138741f, 0.839933f, 2.323170f };

    constexpr float twoPi{ 2.0f * std::numbers::pi_v<float> };
    for (std::size_t i = 0; i < fftSize; ++i)
    {
        const float n{ static_cast<float>(i) };
        const float chirpPhase{ 100.0f * n / sampleRate + 7800.0f * n * n / (2.0f * static_cast<float>(fftSize) * sampleRate) };
        frame.push_back(0.1f * std::sin(twoPi * 125.0f * n / sampleRate) + 0.5f * std::sin(twoPi * 440.0f * n / sampleRate) + 0.3f * std::sin(twoPi * 1800.0f * n / sampleRate + 0.4f) + 0.2f * std::sin(twoPi * chirpPhase));
    }

    const auto& coefficients = mfcc.Compute(frame);

    for (std::size_t m = 0; m < numMelBands; ++m)
        EXPECT_NEAR(mfcc.LogMelEnergies()[m], referenceLogMel[m], 5e-3f) << "band " << m;
    for (std::size_t k = 0; k < numCoefficients; ++k)
        EXPECT_NEAR(coefficients[k], referenceMfcc[k], 5e-3f) << "coefficient " << k;
}

TEST_F(TestMfcc, silent_frame_saturates_at_log_floor)
{
    frame.resize(fftSize, 0.0f);

    const auto& coefficients = mfcc.Compute(frame);

    for (std::size_t m = 0; m < numMelBands; ++m)
        EXPECT_FLOAT_EQ(mfcc.LogMelEnergies()[m], std::log(logFloor));
    EXPECT_NEAR(coefficients[0], std::sqrt(static_cast<float>(numMelBands)) * std::log(logFloor), 1e-3f);
    for (std::size_t k = 1; k < numCoefficients; ++k)
        EXPECT_NEAR(coefficients[k], 0.0f, 1e-3f);
}
