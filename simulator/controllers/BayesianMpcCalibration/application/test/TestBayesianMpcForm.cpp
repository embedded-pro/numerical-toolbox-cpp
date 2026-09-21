#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::controllers::bayesian::BayesianMpcForm;
    namespace field = simulator::controllers::bayesian::field;

    class BayesianMpcFormTest
        : public ::testing::Test
    {
    protected:
        BayesianMpcForm form;
    };
}

TEST_F(BayesianMpcFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.plant.dt, 0.05f, 1e-5f);
    EXPECT_NEAR(config.plant.sigmaQ, 0.01f, 1e-6f);
    EXPECT_NEAR(config.plant.sigmaR, 0.5f, 1e-4f);
    EXPECT_EQ(config.emKf.maxEmIterations, 50u);
    EXPECT_EQ(config.mpcBo.boIterations, 25u);
}

TEST_F(BayesianMpcFormTest, TheIterationCountsAreWholeNumberFields)
{
    EXPECT_EQ(form.Model().Field(field::expectationIterations).kind, ui::model::FieldKind::Integer);
    EXPECT_EQ(form.Model().Field(field::optimisationIterations).kind, ui::model::FieldKind::Integer);
}

TEST_F(BayesianMpcFormTest, TheEvaluationBudgetKeepsTheUpperBoundThePanelGaveIt)
{
    const auto& traits = form.Model().Field(field::optimisationIterations).number;

    EXPECT_NEAR(traits.minimum, 5.0, 1e-9);
    EXPECT_NEAR(traits.maximum, 29.0, 1e-9);
}

TEST_F(BayesianMpcFormTest, EditingAFieldReachesItsOwnSubConfiguration)
{
    form.Model().SetNumber(field::timeStep, 0.2);
    form.Model().SetNumber(field::expectationIterations, 120.0);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.plant.dt, 0.2f, 1e-5f);
    EXPECT_EQ(config.emKf.maxEmIterations, 120u);
}
