#include "simulator/filters/KalmanFilter/application/KalmanForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::filters::kalman::KalmanForm;
    namespace field = simulator::filters::kalman::field;

    class KalmanFormTest
        : public ::testing::Test
    {
    protected:
        KalmanForm form;
    };
}

TEST_F(KalmanFormTest, TheSimulationDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.duration, 10.0f, 1e-3f);
    EXPECT_NEAR(config.dt, 0.01f, 1e-5f);
    EXPECT_NEAR(config.initialTheta, 0.5f, 1e-4f);
    EXPECT_NEAR(config.initialThetaDot, 0.0f, 1e-6f);
}

TEST_F(KalmanFormTest, TheNoiseAndPendulumDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.measurementNoiseStdDev, 0.1f, 1e-4f);
    EXPECT_NEAR(config.processNoiseStdDev, 0.01f, 1e-5f);
    EXPECT_NEAR(config.pendulum.length, 1.0f, 1e-4f);
    EXPECT_NEAR(config.pendulum.mass, 1.0f, 1e-4f);
    EXPECT_NEAR(config.pendulum.damping, 0.1f, 1e-4f);
}

TEST_F(KalmanFormTest, TheInitialAngleKeepsTheTruncatedPiRangeThePanelGaveIt)
{
    const auto& traits = form.Model().Field(field::initialAngle).number;

    EXPECT_NEAR(traits.minimum, -3.14, 1e-9);
    EXPECT_NEAR(traits.maximum, 3.14, 1e-9);
}

TEST_F(KalmanFormTest, EachStepMatchesTheSpinBoxItReplaces)
{
    EXPECT_NEAR(form.Model().Field(field::timeStep).number.step, 0.001, 1e-9);
    EXPECT_NEAR(form.Model().Field(field::processNoise).number.step, 0.001, 1e-9);
    EXPECT_NEAR(form.Model().Field(field::measurementNoise).number.step, 0.01, 1e-9);
    EXPECT_NEAR(form.Model().Field(field::damping).number.step, 0.01, 1e-9);
}

TEST_F(KalmanFormTest, EditingAPendulumFieldReachesTheNestedConfiguration)
{
    form.Model().SetNumber(field::length, 2.5);
    form.Model().SetNumber(field::mass, 3.5);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.pendulum.length, 2.5f, 1e-4f);
    EXPECT_NEAR(config.pendulum.mass, 3.5f, 1e-4f);
}
