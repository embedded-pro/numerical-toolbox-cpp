#include "simulator/controllers/PidController/application/PidForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::controllers::pid::PidForm;
    namespace field = simulator::controllers::pid::field;

    class PidFormTest
        : public ::testing::Test
    {
    protected:
        PidForm form;
    };
}

TEST_F(PidFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.tunings.kp, 1.0f, 1e-4f);
    EXPECT_NEAR(config.tunings.ki, 0.1f, 1e-4f);
    EXPECT_NEAR(config.tunings.kd, 0.05f, 1e-4f);
    EXPECT_NEAR(config.limits.min, -100.0f, 1e-3f);
    EXPECT_NEAR(config.limits.max, 100.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.duration, 20.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.sampleTime, 0.01f, 1e-5f);
}

TEST_F(PidFormTest, ThePlantStartsSecondOrderAsThePanelLeftIt)
{
    EXPECT_EQ(form.Model().Selection(field::plantOrder), 1u);
    EXPECT_EQ(form.PlantDescription(), "2nd Order");
}

TEST_F(PidFormTest, OnlyTheSelectedPlantsFieldsAreVisibleFromTheStart)
{
    EXPECT_FALSE(form.Model().IsVisible(field::gain));
    EXPECT_FALSE(form.Model().IsVisible(field::timeConstant));
    EXPECT_TRUE(form.Model().IsVisible(field::naturalFrequency));
    EXPECT_TRUE(form.Model().IsVisible(field::dampingRatio));

    EXPECT_FALSE(form.Model().IsGroupVisible(field::firstOrder));
    EXPECT_TRUE(form.Model().IsGroupVisible(field::secondOrder));
}

TEST_F(PidFormTest, SwitchingTheOrderSwapsWhichGroupIsShown)
{
    form.Model().SetSelection(field::plantOrder, 0);

    EXPECT_TRUE(form.Model().IsGroupVisible(field::firstOrder));
    EXPECT_FALSE(form.Model().IsGroupVisible(field::secondOrder));
    EXPECT_TRUE(form.Model().IsVisible(field::gain));
    EXPECT_FALSE(form.Model().IsVisible(field::naturalFrequency));
}

TEST_F(PidFormTest, ThePlantIsBuiltFromTheSelectedOrdersOwnFields)
{
    form.Model().SetNumber(field::naturalFrequency, 4.0);
    form.Model().SetNumber(field::dampingRatio, 0.25);

    const auto second = form.CreatePlant()->GetTransferFunction();
    ASSERT_EQ(second.denominator.size(), 3u);
    EXPECT_NEAR(second.denominator[1], 2.0f * 0.25f * 4.0f, 1e-4f);
    EXPECT_NEAR(second.denominator[2], 16.0f, 1e-4f);

    form.Model().SetSelection(field::plantOrder, 0);
    form.Model().SetNumber(field::gain, 3.0);
    form.Model().SetNumber(field::timeConstant, 2.0);

    const auto first = form.CreatePlant()->GetTransferFunction();
    ASSERT_EQ(first.numerator.size(), 1u);
    EXPECT_NEAR(first.numerator[0], 3.0f, 1e-4f);
    ASSERT_EQ(first.denominator.size(), 2u);
    EXPECT_NEAR(first.denominator[0], 2.0f, 1e-4f);
}

TEST_F(PidFormTest, TheHiddenPlantsFieldsKeepTheirValuesAcrossASwitch)
{
    form.Model().SetNumber(field::gain, 7.5);
    form.Model().SetSelection(field::plantOrder, 0);

    EXPECT_NEAR(form.Model().Number(field::gain), 7.5, 1e-9);
}

TEST_F(PidFormTest, TheRootLocusDragWritesTheProportionalGainBackIntoTheForm)
{
    form.SetProportionalGain(12.5f);

    EXPECT_NEAR(form.BuildConfiguration().tunings.kp, 12.5f, 1e-4f);
}

TEST_F(PidFormTest, TheProportionalGainWriteBackIsClampedToTheFieldsRange)
{
    form.SetProportionalGain(250.0f);

    EXPECT_NEAR(form.BuildConfiguration().tunings.kp, 100.0f, 1e-4f);
}
