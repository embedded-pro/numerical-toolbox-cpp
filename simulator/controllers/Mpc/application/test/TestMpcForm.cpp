#include "simulator/controllers/Mpc/application/MpcForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::controllers::mpc::MpcForm;
    namespace field = simulator::controllers::mpc::field;

    class MpcFormTest
        : public ::testing::Test
    {
    protected:
        MpcForm form;
    };
}

TEST_F(MpcFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.weights.stateWeight, 10.0f, 1e-3f);
    EXPECT_NEAR(config.weights.controlWeight, 0.1f, 1e-4f);
    EXPECT_FALSE(config.constraints.enabled);
    EXPECT_NEAR(config.constraints.uMin, -2.0f, 1e-3f);
    EXPECT_NEAR(config.constraints.uMax, 2.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.duration, 10.0f, 1e-3f);
    EXPECT_NEAR(config.simulation.sampleTime, 0.1f, 1e-4f);
    EXPECT_NEAR(config.referencePosition, 1.0f, 1e-3f);
}

TEST_F(MpcFormTest, ThePlantStartsADoubleIntegratorAsThePanelLeftIt)
{
    EXPECT_EQ(form.Model().Selection(field::plantType), 0u);
    EXPECT_EQ(form.PlantDescription(), "Double Integrator");
}

TEST_F(MpcFormTest, ThePlantParametersAreHiddenUntilTheFirstOrderPlantIsChosen)
{
    EXPECT_FALSE(form.Model().IsGroupVisible(field::plantParameters));
    EXPECT_FALSE(form.Model().IsVisible(field::plantGain));

    form.Model().SetSelection(field::plantType, 1);

    EXPECT_TRUE(form.Model().IsGroupVisible(field::plantParameters));
    EXPECT_TRUE(form.Model().IsVisible(field::plantGain));
    EXPECT_TRUE(form.Model().IsVisible(field::plantTimeConstant));
}

TEST_F(MpcFormTest, TheConstraintLimitsStayEnabledWhetherOrNotConstraintsAre)
{
    EXPECT_TRUE(form.Model().IsEnabled(field::controlMinimum));
    EXPECT_TRUE(form.Model().IsEnabled(field::controlMaximum));

    form.Model().SetFlag(field::constraintsEnabled, true);

    EXPECT_TRUE(form.Model().IsEnabled(field::controlMinimum));
    EXPECT_TRUE(form.Model().IsEnabled(field::controlMaximum));
    EXPECT_TRUE(form.BuildConfiguration().constraints.enabled);
}

TEST_F(MpcFormTest, TheDoubleIntegratorIsBuiltAtTheConfiguredSampleTime)
{
    form.Model().SetNumber(field::sampleTime, 0.25);

    const auto plant = form.CreatePlant();

    EXPECT_NEAR(plant.A.at(0, 1), 0.25f, 1e-5f);
    EXPECT_NEAR(plant.B.at(1, 0), 0.25f, 1e-5f);
}

TEST_F(MpcFormTest, TheFirstOrderPlantUsesItsOwnGainAndTimeConstant)
{
    form.Model().SetSelection(field::plantType, 1);
    form.Model().SetNumber(field::plantGain, 3.0);
    form.Model().SetNumber(field::plantTimeConstant, 2.0);
    form.Model().SetNumber(field::sampleTime, 0.1);

    const auto plant = form.CreatePlant();
    const auto expected = simulator::controllers::MakeFirstOrderWithIntegrator(3.0f, 2.0f, 0.1f);

    EXPECT_NEAR(plant.A.at(1, 1), expected.A.at(1, 1), 1e-6f);
    EXPECT_NEAR(plant.B.at(1, 0), expected.B.at(1, 0), 1e-6f);
}
