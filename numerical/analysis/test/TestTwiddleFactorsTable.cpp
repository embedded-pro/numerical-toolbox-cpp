#include "numerical/analysis/TwiddleFactorsTable.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gmock/gmock.h>

namespace
{
    class TestTwiddleFactorsTable
        : public ::testing::Test
    {
    protected:
        static float Magnitude(const math::Complex<float>& value)
        {
            return std::sqrt(value.Real() * value.Real() + value.Imaginary() * value.Imaginary());
        }

        analysis::TwiddleFactorsTable<float, 8> factors;
    };
}

TEST_F(TestTwiddleFactorsTable, TheFirstFactorIsUnity)
{
    EXPECT_NEAR(factors[0].Real(), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(factors[0].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestTwiddleFactorsTable, EveryFactorLiesOnTheUnitCircle)
{
    for (std::size_t k = 0; k < 8; ++k)
        EXPECT_NEAR(Magnitude(factors[k]), 1.0f, math::Tolerance<float>());
}

TEST_F(TestTwiddleFactorsTable, TheFactorsMatchTheirClosedForm)
{
    for (std::size_t k = 0; k < 8; ++k)
    {
        const auto angle = -std::numbers::pi_v<float> * static_cast<float>(k) / 8.0f;

        EXPECT_NEAR(factors[k].Real(), std::cos(angle), math::Tolerance<float>());
        EXPECT_NEAR(factors[k].Imaginary(), std::sin(angle), math::Tolerance<float>());
    }
}

TEST_F(TestTwiddleFactorsTable, TheMidpointIsANegativeQuarterTurn)
{
    EXPECT_NEAR(factors[4].Real(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(factors[4].Imaginary(), -1.0f, math::Tolerance<float>());
}

TEST_F(TestTwiddleFactorsTable, TheRealPartDescendsMonotonicallyAcrossTheHalfTurn)
{
    for (std::size_t k = 1; k < 8; ++k)
        EXPECT_LT(factors[k].Real(), factors[k - 1].Real());
}

TEST_F(TestTwiddleFactorsTable, TheImaginaryPartTurnsAtTheQuarterTurn)
{
    for (std::size_t k = 1; k <= 4; ++k)
        EXPECT_LT(factors[k].Imaginary(), factors[k - 1].Imaginary());

    for (std::size_t k = 5; k < 8; ++k)
        EXPECT_GT(factors[k].Imaginary(), factors[k - 1].Imaginary());
}

TEST_F(TestTwiddleFactorsTable, ASmallerTableSpansTheSameHalfTurn)
{
    analysis::TwiddleFactorsTable<float, 2> small;

    EXPECT_NEAR(small[0].Real(), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(small[1].Real(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(small[1].Imaginary(), -1.0f, math::Tolerance<float>());
}

TEST_F(TestTwiddleFactorsTable, FactorsAreAddressableThroughTheInterface)
{
    analysis::TwiddleFactors<float, 8>& asInterface = factors;

    EXPECT_NEAR(asInterface[0].Real(), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(asInterface[4].Imaginary(), -1.0f, math::Tolerance<float>());
}
