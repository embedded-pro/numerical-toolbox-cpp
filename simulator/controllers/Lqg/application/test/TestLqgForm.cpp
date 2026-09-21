#include "simulator/controllers/Lqg/application/LqgForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::controllers::lqg::LqgForm;
    namespace field = simulator::controllers::lqg::field;

    class LqgFormTest
        : public ::testing::Test
    {
    protected:
        LqgForm form;
    };
}

// Transcribed from LqgConfigurationPanel.cpp as it stood before deletion.
TEST_F(LqgFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.weights.stateWeight, 10.0f, 1e-4f);
    EXPECT_NEAR(config.weights.controlWeight, 0.1f, 1e-5f);
    EXPECT_NEAR(config.noise.processNoise, 0.1f, 1e-5f);
    EXPECT_NEAR(config.noise.measurementNoise, 1.0f, 1e-5f);
    EXPECT_NEAR(config.simulation.sampleTime, 0.1f, 1e-5f);
    EXPECT_NEAR(config.simulation.duration, 10.0f, 1e-4f);
    EXPECT_NEAR(config.initialPosition, 1.0f, 1e-5f);
}

// The configuration is nested three groups deep, so every field has to land in its own sub-struct
// rather than a flat bag.
TEST_F(LqgFormTest, EachFieldLandsInItsOwnSubStructure)
{
    form.Model().SetNumber(field::stateWeight, 42.0);
    form.Model().SetNumber(field::processNoise, 0.25);
    form.Model().SetNumber(field::duration, 33.0);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.weights.stateWeight, 42.0f, 1e-4f);
    EXPECT_NEAR(config.noise.processNoise, 0.25f, 1e-5f);
    EXPECT_NEAR(config.simulation.duration, 33.0f, 1e-4f);
}

// A discrete double integrator at the configured step: the plant is built from a form value, and
// no matrix type appears anywhere in the UI library.
TEST_F(LqgFormTest, ThePlantIsDiscretisedAtTheConfiguredSampleTime)
{
    form.Model().SetNumber(field::sampleTime, 0.25);

    const auto plant = form.CreatePlant();

    EXPECT_NEAR(plant.A.at(0, 0), 1.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(0, 1), 0.25f, 1e-6f);
    EXPECT_NEAR(plant.A.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(plant.A.at(1, 1), 1.0f, 1e-6f);
    EXPECT_NEAR(plant.B.at(1, 0), 0.25f, 1e-6f);
    EXPECT_NEAR(plant.C.at(0, 0), 1.0f, 1e-6f);
}
