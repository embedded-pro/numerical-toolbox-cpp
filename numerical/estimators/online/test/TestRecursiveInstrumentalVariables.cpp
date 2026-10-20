#include "numerical/estimators/online/RecursiveInstrumentalVariables.hpp"
#include "numerical/estimators/online/RecursiveLeastSquares.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <random>

namespace
{
    class TestRecursiveInstrumentalVariables
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Features = 2;

        using Riv = estimators::RecursiveInstrumentalVariables<float, Features>;
        using Rls = estimators::RecursiveLeastSquares<float, Features>;
        using Input = Riv::InputMatrix;
        using Output = math::Matrix<float, 1, 1>;

        static constexpr float trueIntercept = 0.5f;
        static constexpr float trueSlope = 2.0f;

        Input MakeInput(float bias, float x)
        {
            Input result;
            result.at(0, 0) = bias;
            result.at(1, 0) = x;
            return result;
        }

        Output MakeOutput(float y)
        {
            Output result;
            result.at(0, 0) = y;
            return result;
        }

        std::mt19937 engine{ 5u };
        std::normal_distribution<float> unit{ 0.0f, 1.0f };
    };
}

TEST_F(TestRecursiveInstrumentalVariables, MatchesRecursiveLeastSquaresWhenTheRegressorIsItsOwnInstrument)
{
    Riv riv{ 1000.0f, 0.99f };
    Rls rls{ 1000.0f, 0.99f };
    const auto seed = MakeInput(1.0f, 1.0f);
    riv.SetCoefficients(seed);
    rls.SetCoefficients(seed);
    Riv::EstimationMetrics rivMetrics{};
    Rls::EstimationMetrics rlsMetrics{};

    for (int i{ 0 }; i != 200; ++i)
    {
        const float x = std::sin(0.1f * static_cast<float>(i));
        const auto regressor = MakeInput(1.0f, x);
        const auto output = MakeOutput(trueIntercept + trueSlope * x + 0.1f * unit(engine));
        rivMetrics = riv.Update(regressor, regressor, output);
        rlsMetrics = rls.Update(regressor, output);
    }

    EXPECT_NEAR(riv.Coefficients().at(0, 0), rls.Coefficients().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(riv.Coefficients().at(1, 0), rls.Coefficients().at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(rivMetrics.innovation, rlsMetrics.innovation, math::Tolerance<float>());
    EXPECT_NEAR(rivMetrics.residual, rlsMetrics.residual, math::Tolerance<float>());
    EXPECT_NEAR(rivMetrics.uncertainty, rlsMetrics.uncertainty, math::Tolerance<float>());
}

TEST_F(TestRecursiveInstrumentalVariables, RecoversTheSlopeWhenOnlyTheRegressorIsNoisy)
{
    Riv riv{ 1000.0f, 1.0f };

    for (int i{ 0 }; i != 4000; ++i)
    {
        const float x = unit(engine);
        const float observed = x + 0.5f * unit(engine);
        riv.Update(MakeInput(1.0f, observed), MakeInput(1.0f, x), MakeOutput(trueIntercept + trueSlope * x));
    }

    EXPECT_NEAR(riv.Coefficients().at(0, 0), trueIntercept, 0.05f);
    EXPECT_NEAR(riv.Coefficients().at(1, 0), trueSlope, 0.1f);
}
