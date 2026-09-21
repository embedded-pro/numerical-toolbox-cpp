#include "simulator/controllers/LqrCartPole/application/LqrForm.hpp"
#include <gmock/gmock.h>
#include <numbers>

namespace
{
    using simulator::controllers::lqr::LqrForm;
    namespace field = simulator::controllers::lqr::field;

    class LqrFormTest
        : public ::testing::Test
    {
    protected:
        LqrForm form;
    };
}

TEST_F(LqrFormTest, ThePlantDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.plantParams.cartMass, 1.0f, 1e-3f);
    EXPECT_NEAR(config.plantParams.poleMass, 0.1f, 1e-4f);
    EXPECT_NEAR(config.plantParams.poleLength, 0.5f, 1e-3f);
    EXPECT_NEAR(config.plantParams.gravity, 9.81f, 1e-3f);
    EXPECT_NEAR(config.plantParams.cartFriction, 0.1f, 1e-4f);
    EXPECT_NEAR(config.plantParams.trackLimit, 2.4f, 1e-3f);
}

TEST_F(LqrFormTest, TheWeightAndSimulationDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.weights.qX, 1.0f, 1e-4f);
    EXPECT_NEAR(config.weights.qXDot, 1.0f, 1e-4f);
    EXPECT_NEAR(config.weights.qTheta, 100.0f, 1e-3f);
    EXPECT_NEAR(config.weights.qThetaDot, 10.0f, 1e-3f);
    EXPECT_NEAR(config.weights.rForce, 0.01f, 1e-6f);
    EXPECT_NEAR(config.simulation.dt, 0.01f, 1e-5f);
    EXPECT_NEAR(config.simulation.forceLimit, 50.0f, 1e-3f);
}

TEST_F(LqrFormTest, TheAngleReadOutsAreDegreesWhileTheSimulatorStaysInRadians)
{
    constexpr auto quarterTurn = std::numbers::pi_v<float> / 2.0f;

    form.SetState(0.0f, 0.0f, quarterTurn, -quarterTurn, 0.0f);

    EXPECT_NEAR(form.Model().Number(field::readOutAngle), 90.0, 1e-3);
    EXPECT_NEAR(form.Model().Number(field::readOutAngularRate), -90.0, 1e-3);
}

TEST_F(LqrFormTest, TheLinearReadOutsPassThroughUnconverted)
{
    form.SetState(1.25f, -0.5f, 0.0f, 0.0f, 12.5f);

    EXPECT_NEAR(form.Model().Number(field::readOutPosition), 1.25, 1e-6);
    EXPECT_NEAR(form.Model().Number(field::readOutVelocity), -0.5, 1e-6);
    EXPECT_NEAR(form.Model().Number(field::readOutForce), 12.5, 1e-6);
}

TEST_F(LqrFormTest, TheStateReadOutsAreNotEditable)
{
    EXPECT_EQ(form.Model().Field(field::readOutPosition).kind, ui::model::FieldKind::ReadOut);
    EXPECT_EQ(form.Model().Field(field::readOutAngle).kind, ui::model::FieldKind::ReadOut);
    EXPECT_EQ(form.Model().Field(field::readOutForce).kind, ui::model::FieldKind::ReadOut);
}

TEST_F(LqrFormTest, TheFiveControlActionsAreAllOffered)
{
    EXPECT_EQ(form.Model().Spec().actions.size(), 5u);
}
