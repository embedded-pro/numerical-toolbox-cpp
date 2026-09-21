#include "simulator/analysis/FastFourierTransform/application/FftForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::analysis::FftForm;
    namespace field = simulator::analysis::field;

    class FftFormTest
        : public ::testing::Test
    {
    protected:
        FftForm form;
    };
}

TEST_F(FftFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.fftSize, 1024u);
    EXPECT_NEAR(config.sampleRateHz, 44100.0f, 1e-3f);
    EXPECT_EQ(config.windowType, simulator::analysis::WindowType::Rectangular);
}

TEST_F(FftFormTest, TheDefaultSizeIsSelectedByItsValueNotItsPosition)
{
    EXPECT_EQ(form.Model().Selection(field::fftSize), 4u);
}

TEST_F(FftFormTest, EverySupportedSizeIsOfferedAndReadsBackAsItself)
{
    const auto sizes = simulator::analysis::FftSimulator::SupportedFftSizes();
    ASSERT_EQ(form.Model().Field(field::fftSize).options.size(), sizes.size());

    for (std::size_t i = 0; i < sizes.size(); ++i)
    {
        form.Model().SetSelection(field::fftSize, i);
        EXPECT_EQ(form.BuildConfiguration().fftSize, sizes[i]);
    }
}

TEST_F(FftFormTest, TheWindowTypeComesFromTheOptionDataNotItsPosition)
{
    form.Model().SetSelection(field::windowType, 3);

    EXPECT_EQ(form.BuildConfiguration().windowType, simulator::analysis::WindowType::Blackman);
}

TEST_F(FftFormTest, TheSignalComponentsAreSeededAsThePanelSeededThem)
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

TEST_F(FftFormTest, EditingTheTableReachesTheConfiguration)
{
    auto& table = form.Model().Table(0);
    table.SetCell(1, 0, 6000.0);
    table.SetCell(1, 1, 0.75);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.signalComponents[1].frequencyHz, 6000.0f, 1e-3f);
    EXPECT_NEAR(config.signalComponents[1].amplitude, 0.75f, 1e-4f);
}
