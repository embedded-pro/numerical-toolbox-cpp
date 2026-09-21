#include "simulator/filters/IirFilter/application/IirForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::filters::iir::IirForm;
    namespace field = simulator::filters::iir::field;

    class IirFormTest
        : public ::testing::Test
    {
    protected:
        IirForm form;
    };
}

TEST_F(IirFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.filter.type, simulator::filters::iir::FilterType::LowPass);
    EXPECT_NEAR(config.filter.cutoffHz, 1000.0f, 1e-3f);
    EXPECT_NEAR(config.filter.qualityFactor, 0.707f, 1e-4f);
    EXPECT_NEAR(config.filter.sampleRateHz, 8000.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.duration, 0.05f, 1e-5f);
}

TEST_F(IirFormTest, TheSampleRateIsSharedByTheFilterAndTheSimulation)
{
    form.Model().SetNumber(field::sampleRate, 44100.0);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.filter.sampleRateHz, 44100.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.sampleRateHz, 44100.0f, 1e-3f);
}

TEST_F(IirFormTest, TheFilterTypeComesFromTheOptionDataNotItsPosition)
{
    form.Model().SetSelection(field::filterType, 1);
    EXPECT_EQ(form.BuildConfiguration().filter.type, simulator::filters::iir::FilterType::HighPass);

    form.Model().SetSelection(field::filterType, 2);
    EXPECT_EQ(form.BuildConfiguration().filter.type, simulator::filters::iir::FilterType::BandPass);
}

TEST_F(IirFormTest, TheSignalComponentsAreSeededAsThePanelSeededThem)
{
    const auto config = form.BuildConfiguration();

    ASSERT_EQ(config.simulation.signalComponents.size(), 2u);
    EXPECT_NEAR(config.simulation.signalComponents[0].frequencyHz, 200.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.signalComponents[0].amplitude, 1.0f, 1e-4f);
    EXPECT_NEAR(config.simulation.signalComponents[1].frequencyHz, 2000.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.signalComponents[1].amplitude, 0.5f, 1e-4f);
}

TEST_F(IirFormTest, EditingTheTableReachesTheConfiguration)
{
    auto& table = form.Model().Table(0);
    table.SetCell(0, 0, 440.0);
    table.SetCell(0, 1, 0.25);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.simulation.signalComponents[0].frequencyHz, 440.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.signalComponents[0].amplitude, 0.25f, 1e-4f);
}

TEST_F(IirFormTest, RemovingEveryComponentLeavesNoneRatherThanAPhantomRow)
{
    auto& table = form.Model().Table(0);
    while (table.RemoveLastRow())
    {
    }

    EXPECT_TRUE(form.BuildConfiguration().simulation.signalComponents.empty());
}
