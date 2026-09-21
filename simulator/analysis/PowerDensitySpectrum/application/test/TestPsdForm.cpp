#include "simulator/analysis/PowerDensitySpectrum/application/PsdForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::analysis::psd::PsdForm;
    namespace field = simulator::analysis::psd::field;

    class PsdFormTest
        : public ::testing::Test
    {
    protected:
        PsdForm form;
    };
}

TEST_F(PsdFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.inputSize, 1024u);
    EXPECT_EQ(config.segmentSize, 256u);
    EXPECT_EQ(config.overlapPercent, 50u);
    EXPECT_NEAR(config.sampleRateHz, 44100.0f, 1e-3f);
    EXPECT_EQ(config.windowType, simulator::analysis::psd::WindowType::Hamming);
    EXPECT_EQ(config.noise.type, simulator::analysis::psd::NoiseType::WhiteGaussian);
    EXPECT_NEAR(config.noise.amplitude, 0.1f, 1e-5f);
}

TEST_F(PsdFormTest, TheNoiseAmplitudeIsEnabledFromTheStartRatherThanOnlyAfterTheFirstChange)
{
    EXPECT_TRUE(form.Model().IsEnabled(field::noiseAmplitude));
}

TEST_F(PsdFormTest, SelectingNoNoiseDisablesTheAmplitudeAndSelectingItBackReEnablesIt)
{
    form.Model().SetSelection(field::noiseType, 0);
    EXPECT_FALSE(form.Model().IsEnabled(field::noiseAmplitude));

    form.Model().SetSelection(field::noiseType, 1);
    EXPECT_TRUE(form.Model().IsEnabled(field::noiseAmplitude));
}

TEST_F(PsdFormTest, TheAmplitudeIsReadWhileDisabledExactlyAsThePanelReadIt)
{
    form.Model().SetSelection(field::noiseType, 0);

    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.noise.type, simulator::analysis::psd::NoiseType::None);
    EXPECT_NEAR(config.noise.amplitude, 0.1f, 1e-5f);
}

TEST_F(PsdFormTest, EverySupportedSegmentSizeAndOverlapIsOfferedAndReadsBackAsItself)
{
    const auto segments = simulator::analysis::psd::PsdSimulator::SupportedSegmentSizes();
    ASSERT_EQ(form.Model().Field(field::segmentSize).options.size(), segments.size());

    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        form.Model().SetSelection(field::segmentSize, i);
        EXPECT_EQ(form.BuildConfiguration().segmentSize, segments[i]);
    }

    const auto overlaps = simulator::analysis::psd::PsdSimulator::SupportedOverlapValues();
    ASSERT_EQ(form.Model().Field(field::overlap).options.size(), overlaps.size());

    for (std::size_t i = 0; i < overlaps.size(); ++i)
    {
        form.Model().SetSelection(field::overlap, i);
        EXPECT_EQ(form.BuildConfiguration().overlapPercent, overlaps[i]);
    }
}

TEST_F(PsdFormTest, TheInputSizeIsWholeSamplesRatherThanARoundedDouble)
{
    form.Model().SetNumber(field::inputSize, 2048.0);

    EXPECT_EQ(form.BuildConfiguration().inputSize, 2048u);
}

TEST_F(PsdFormTest, TheSignalComponentsAreSeededAsThePanelSeededThem)
{
    const auto config = form.BuildConfiguration();

    ASSERT_EQ(config.signalComponents.size(), 3u);
    EXPECT_NEAR(config.signalComponents[0].frequencyHz, 1000.0f, 1e-3f);
    EXPECT_NEAR(config.signalComponents[0].amplitude, 0.15f, 1e-4f);
    EXPECT_NEAR(config.signalComponents[1].frequencyHz, 5000.0f, 1e-3f);
    EXPECT_NEAR(config.signalComponents[1].amplitude, 0.5f, 1e-4f);
    EXPECT_NEAR(config.signalComponents[2].frequencyHz, 12000.0f, 1e-3f);
    EXPECT_NEAR(config.signalComponents[2].amplitude, 0.25f, 1e-4f);
}
