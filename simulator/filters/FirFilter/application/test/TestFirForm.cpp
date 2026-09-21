#include "simulator/filters/FirFilter/application/FirForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::filters::fir::FirForm;
    namespace field = simulator::filters::fir::field;

    class FirFormTest
        : public ::testing::Test
    {
    protected:
        FirForm form;
    };
}

TEST_F(FirFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_EQ(config.filter.type, simulator::filters::fir::FilterType::LowPass);
    EXPECT_NEAR(config.filter.cutoffHz, 1000.0f, 1e-3f);
    EXPECT_NEAR(config.filter.cutoffHighHz, 3000.0f, 1e-3f);
    EXPECT_EQ(config.filter.order, 31u);
    EXPECT_NEAR(config.filter.sampleRateHz, 8000.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.duration, 0.05f, 1e-5f);
}

TEST_F(FirFormTest, TheUpperCutoffIsEnabledOnlyForABandPass)
{
    EXPECT_FALSE(form.Model().IsEnabled(field::cutoffHigh));

    form.Model().SetSelection(field::filterType, 2);
    EXPECT_TRUE(form.Model().IsEnabled(field::cutoffHigh));

    form.Model().SetSelection(field::filterType, 1);
    EXPECT_FALSE(form.Model().IsEnabled(field::cutoffHigh));
}

TEST_F(FirFormTest, TheUpperCutoffIsStillReadWhileDisabled)
{
    ASSERT_FALSE(form.Model().IsEnabled(field::cutoffHigh));
    form.Model().SetNumber(field::cutoffHigh, 5000.0);

    EXPECT_NEAR(form.BuildConfiguration().filter.cutoffHighHz, 5000.0f, 1e-3f);
}

TEST_F(FirFormTest, TheOrderIsAnIntegerCount)
{
    form.Model().SetNumber(field::order, 63.0);

    EXPECT_EQ(form.BuildConfiguration().filter.order, 63u);
}

TEST_F(FirFormTest, TheSignalComponentsAreSeededAsThePanelSeededThem)
{
    const auto config = form.BuildConfiguration();

    ASSERT_EQ(config.simulation.signalComponents.size(), 2u);
    EXPECT_NEAR(config.simulation.signalComponents[0].frequencyHz, 200.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.signalComponents[1].amplitude, 0.5f, 1e-4f);
}
