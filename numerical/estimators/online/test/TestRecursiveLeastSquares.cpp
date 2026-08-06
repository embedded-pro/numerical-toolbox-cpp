#include "numerical/estimators/online/RecursiveLeastSquares.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestRecursiveLeastSquares
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Features = 2;

        using Rls = estimators::RecursiveLeastSquares<float, Features>;
        using Input = Rls::InputMatrix;
        using Output = math::Matrix<float, 1, 1>;

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
    };
}

TEST_F(TestRecursiveLeastSquares, ConvergesWithMakeRegressorToLinearRelationship)
{
    constexpr float trueIntercept = 2.0f;
    constexpr float trueSlope = 3.0f;

    Rls rls{ 1000.0f, 0.99f };
    Input regressor;
    Output output;
    Rls::EstimationMetrics metrics;

    for (int i{ 1 }; i <= 100; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        Rls::MakeRegressor(regressor, x);
        output.at(0, 0) = trueIntercept + trueSlope * x;
        metrics = rls.Update(regressor, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), trueIntercept, 0.1f);
    EXPECT_NEAR(coef.at(1, 0), trueSlope, 0.1f);

    auto state = Rls::EvaluateConvergence(metrics, 0.1f, 1.0f);
    EXPECT_EQ(state, estimators::State::converged);
}

TEST_F(TestRecursiveLeastSquares, ConvergesWithNoisyData)
{
    constexpr float trueIntercept = 1.5f;
    constexpr float trueSlope = 2.5f;

    Rls rls{ 1000.0f, 0.99f };
    uint32_t seed{ 12345 };
    auto nextNoise = [&seed]() -> float
    {
        seed = seed * 1103515245u + 12345u;
        return (static_cast<float>(seed % 1000u) / 1000.0f - 0.5f) * 0.2f;
    };

    for (int i{ 1 }; i <= 200; ++i)
    {
        float x = static_cast<float>(i) * 0.05f;
        rls.Update(MakeInput(1.0f, x), MakeOutput(trueIntercept + trueSlope * x + nextNoise()));
    }

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), trueIntercept, 0.2f);
    EXPECT_NEAR(coef.at(1, 0), trueSlope, 0.2f);
}

TEST_F(TestRecursiveLeastSquares, TracksTimeVaryingSystem)
{
    Rls rls{ 1000.0f, 0.95f };

    for (int i{ 1 }; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        rls.Update(MakeInput(1.0f, x), MakeOutput(1.0f + 2.0f * x));
    }

    auto metricsAfterChange = rls.Update(MakeInput(1.0f, 0.1f), MakeOutput(3.0f + 4.0f * 0.1f));
    EXPECT_EQ(Rls::EvaluateConvergence(metricsAfterChange, 0.1f, 1.0f), estimators::State::unstable);

    Rls::EstimationMetrics metrics;
    for (int i{ 2 }; i <= 200; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        metrics = rls.Update(MakeInput(1.0f, x), MakeOutput(3.0f + 4.0f * x));
    }

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), 3.0f, 0.5f);
    EXPECT_NEAR(coef.at(1, 0), 4.0f, 0.5f);

    EXPECT_EQ(Rls::EvaluateConvergence(metrics, 0.5f, 10.0f), estimators::State::converged);
}

TEST_F(TestRecursiveLeastSquares, InitializesWithDefaultCovariance)
{
    Rls rls{ 1.0f };
    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(coef.at(1, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveLeastSquares, SetCoefficientsSeedsInitialTheta)
{
    Rls rls{ 0.99f };
    Rls::CoefficientsMatrix initial;
    initial.at(0, 0) = 5.0f;
    initial.at(1, 0) = 3.0f;
    rls.SetCoefficients(initial);

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), 5.0f, math::Tolerance<float>());
    EXPECT_NEAR(coef.at(1, 0), 3.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveLeastSquares, SingleFeatureConvergesToSlope)
{
    using SingleFeatureRls = estimators::RecursiveLeastSquares<float, 1>;
    SingleFeatureRls rls{ 1000.0f, 1.0f };

    for (int i{ 1 }; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.2f;
        math::Matrix<float, 1, 1> input;
        input.at(0, 0) = x;
        math::Matrix<float, 1, 1> output;
        output.at(0, 0) = 5.0f * x;
        rls.Update(input, output);
    }

    EXPECT_NEAR(rls.Coefficients().at(0, 0), 5.0f, 0.1f);
}

TEST_F(TestRecursiveLeastSquares, ThreeFeaturesConvergesToPlane)
{
    using MultiRls = estimators::RecursiveLeastSquares<float, 3>;
    MultiRls rls{ 1000.0f, 0.99f };

    for (int i{ 1 }; i <= 200; ++i)
    {
        float x1 = static_cast<float>(i % 20) * 0.1f + 0.1f;
        float x2 = static_cast<float>((i * 7) % 20) * 0.1f + 0.1f;

        math::Matrix<float, 3, 1> input;
        input.at(0, 0) = 1.0f;
        input.at(1, 0) = x1;
        input.at(2, 0) = x2;

        math::Matrix<float, 1, 1> output;
        output.at(0, 0) = 1.0f + 2.0f * x1 + 3.0f * x2;

        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), 1.0f, 0.2f);
    EXPECT_NEAR(coef.at(1, 0), 2.0f, 0.2f);
    EXPECT_NEAR(coef.at(2, 0), 3.0f, 0.2f);
}

TEST_F(TestRecursiveLeastSquares, ForgettingFactorOneConvergesMonotonically)
{
    Rls rls{ 1000.0f, 1.0f };
    Rls::EstimationMetrics metrics;

    for (int i{ 1 }; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        metrics = rls.Update(MakeInput(1.0f, x), MakeOutput(2.0f + x));
    }

    const auto& coef = rls.Coefficients();
    EXPECT_NEAR(coef.at(0, 0), 2.0f, 0.1f);
    EXPECT_NEAR(coef.at(1, 0), 1.0f, 0.1f);
    EXPECT_EQ(Rls::EvaluateConvergence(metrics, 0.1f, 0.1f), estimators::State::converged);
}

TEST_F(TestRecursiveLeastSquares, MakeRegressorSetsBiasAndSample)
{
    Input regressor;
    Rls::MakeRegressor(regressor, 3.5f);
    EXPECT_NEAR(regressor.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(regressor.at(1, 0), 3.5f, math::Tolerance<float>());
}

TEST_F(TestRecursiveLeastSquares, MakeRegressorWithThreeFeaturesSetsBiasAndSamples)
{
    using MultiRls = estimators::RecursiveLeastSquares<float, 3>;
    math::Matrix<float, 3, 1> regressor;
    MultiRls::MakeRegressor(regressor, 1.5f, 2.5f);
    EXPECT_NEAR(regressor.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(regressor.at(1, 0), 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(regressor.at(2, 0), 2.5f, math::Tolerance<float>());
}

TEST_F(TestRecursiveLeastSquares, MakeRegressorReturnsReference)
{
    Input regressor;
    auto& returned = Rls::MakeRegressor(regressor, 1.0f);
    EXPECT_EQ(&returned, &regressor);
}

TEST_F(TestRecursiveLeastSquares, DeterministicOutputForIdenticalInputSequences)
{
    Rls rls1{ 1000.0f, 0.99f };
    Rls rls2{ 1000.0f, 0.99f };

    for (int i{ 1 }; i <= 30; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        auto m1 = rls1.Update(MakeInput(1.0f, x), MakeOutput(2.0f + 3.0f * x));
        auto m2 = rls2.Update(MakeInput(1.0f, x), MakeOutput(2.0f + 3.0f * x));
        EXPECT_FLOAT_EQ(m1.innovation, m2.innovation);
        EXPECT_FLOAT_EQ(m1.residual, m2.residual);
        EXPECT_FLOAT_EQ(m1.uncertainty, m2.uncertainty);
    }
}

TEST_F(TestRecursiveLeastSquares, TwoInterleavedInstancesDontInterfere)
{
    Rls rlsA{ 1000.0f, 0.99f };
    Rls rlsB{ 1000.0f, 0.99f };

    for (int i{ 1 }; i <= 50; ++i)
    {
        float xA = static_cast<float>(i) * 0.1f;
        float xB = static_cast<float>(i) * 0.2f;
        rlsA.Update(MakeInput(1.0f, xA), MakeOutput(1.0f + xA));
        rlsB.Update(MakeInput(1.0f, xB), MakeOutput(5.0f + 2.0f * xB));
    }

    const auto& coefA = rlsA.Coefficients();
    const auto& coefB = rlsB.Coefficients();
    EXPECT_NEAR(coefA.at(0, 0), 1.0f, 0.2f);
    EXPECT_NEAR(coefA.at(1, 0), 1.0f, 0.2f);
    EXPECT_NEAR(coefB.at(0, 0), 5.0f, 0.2f);
    EXPECT_NEAR(coefB.at(1, 0), 2.0f, 0.2f);
}

TEST_F(TestRecursiveLeastSquares, UncertaintyDecreasesOnNoiselessData)
{
    Rls rls{ 1000.0f, 1.0f };

    auto first = rls.Update(MakeInput(1.0f, 0.1f), MakeOutput(2.0f + 0.1f));

    float prevUncertainty = first.uncertainty;
    for (int i{ 2 }; i <= 30; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        auto m = rls.Update(MakeInput(1.0f, x), MakeOutput(2.0f + x));
        EXPECT_LT(m.uncertainty, prevUncertainty);
        prevUncertainty = m.uncertainty;
    }
}

TEST_F(TestRecursiveLeastSquares, UncertaintyRemainsPositive)
{
    Rls rls{ 1000.0f, 1.0f };

    for (int i{ 1 }; i <= 100; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        auto m = rls.Update(MakeInput(1.0f, x), MakeOutput(2.0f + 3.0f * x));
        EXPECT_GT(m.uncertainty, 0.0f);
    }
}

TEST_F(TestRecursiveLeastSquares, ResidualSmallerThanInnovationAfterUpdate)
{
    Rls rls{ 1000.0f, 0.99f };

    for (int i{ 1 }; i <= 20; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        auto m = rls.Update(MakeInput(1.0f, x), MakeOutput(1.0f + 2.0f * x));
        EXPECT_LE(std::abs(m.residual), std::abs(m.innovation) + math::Tolerance<float>());
    }
}
