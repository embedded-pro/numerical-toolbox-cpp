#include "simulator/estimators/RecursiveLeastSquares/application/RlsForm.hpp"
#include <gmock/gmock.h>

namespace
{
    using simulator::estimators::rls::RlsForm;
    namespace field = simulator::estimators::rls::field;

    class RlsFormTest
        : public ::testing::Test
    {
    protected:
        RlsForm form;
    };
}

TEST_F(RlsFormTest, TheDefaultsMatchThePanelThisReplaced)
{
    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.rls.forgettingFactor, 0.99f, 1e-4f);
    EXPECT_NEAR(config.rls.initialCovariance, 1000.0f, 1e-2f);
    EXPECT_EQ(config.rls.numSamples, 200u);
    EXPECT_NEAR(config.rls.noiseAmplitude, 0.1f, 1e-4f);
}

TEST_F(RlsFormTest, TheThreeCoefficientsAreSeparateFieldsInTheirDeclaredOrder)
{
    const auto config = form.BuildConfiguration();

    ASSERT_EQ(config.rls.trueCoefficients.size(), 3u);
    EXPECT_NEAR(config.rls.trueCoefficients[0], 2.0f, 1e-4f);
    EXPECT_NEAR(config.rls.trueCoefficients[1], -1.5f, 1e-4f);
    EXPECT_NEAR(config.rls.trueCoefficients[2], 0.8f, 1e-4f);
}

TEST_F(RlsFormTest, EditingACoefficientReachesItsOwnSlotAndNoOther)
{
    form.Model().SetNumber(field::firstCoefficient, 4.25);

    const auto config = form.BuildConfiguration();

    EXPECT_NEAR(config.rls.trueCoefficients[0], 2.0f, 1e-4f);
    EXPECT_NEAR(config.rls.trueCoefficients[1], 4.25f, 1e-4f);
    EXPECT_NEAR(config.rls.trueCoefficients[2], 0.8f, 1e-4f);
}

TEST_F(RlsFormTest, TheSampleCountIsAWholeNumberField)
{
    EXPECT_EQ(form.Model().Field(field::sampleCount).kind, ui::model::FieldKind::Integer);

    form.Model().SetNumber(field::sampleCount, 1500.0);

    EXPECT_EQ(form.BuildConfiguration().rls.numSamples, 1500u);
}

TEST_F(RlsFormTest, TheForgettingFactorKeepsTheNarrowRangeThePanelGaveIt)
{
    const auto& traits = form.Model().Field(field::forgettingFactor).number;

    EXPECT_NEAR(traits.minimum, 0.9, 1e-9);
    EXPECT_NEAR(traits.maximum, 1.0, 1e-9);
    EXPECT_NEAR(traits.step, 0.005, 1e-9);
    EXPECT_EQ(traits.decimals, 3);
}
