#include "numerical/estimators/RecursiveLeastSquares.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    bool AreFloatsNear(T a, T b, float epsilon = 1e-3f)
    {
        if constexpr (std::is_same_v<T, float>)
            return std::abs(a - b) < epsilon;
        else
            return std::abs(a.ToFloat() - b.ToFloat()) < epsilon;
    }

    template<typename T>
    class RecursiveLeastSquaresTest
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Features = 2;

        using EstimatorType = estimators::RecursiveLeastSquares<T, Features>;
        using InputType = typename EstimatorType::InputMatrix;
        using OutputType = math::Matrix<T, 1, 1>;

        static T MakeValue(float f)
        {
            return T(f);
        }

        InputType MakeInput(float bias, float x)
        {
            InputType result;
            result.at(0, 0) = MakeValue(bias);
            result.at(1, 0) = MakeValue(x);
            return result;
        }

        OutputType MakeOutput(float y)
        {
            OutputType result;
            result.at(0, 0) = MakeValue(y);
            return result;
        }
    };

    using TestTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(RecursiveLeastSquaresTest, TestTypes);
}

TYPED_TEST(RecursiveLeastSquaresTest, ConvergesToLinearRelationship)
{
    constexpr float trueIntercept = 2.0f;
    constexpr float trueSlope = 3.0f;

    typename TestFixture::EstimatorType rls(1000.0f, 0.99f);

    for (int i = 1; i <= 100; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        float y = trueIntercept + trueSlope * x;

        auto input = this->MakeInput(1.0f, x);
        auto output = this->MakeOutput(y);
        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(trueIntercept), 0.1f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(trueSlope), 0.1f));
}

TYPED_TEST(RecursiveLeastSquaresTest, ConvergesWithNoisyData)
{
    constexpr float trueIntercept = 1.5f;
    constexpr float trueSlope = 2.5f;

    typename TestFixture::EstimatorType rls(1000.0f, 0.99f);

    uint32_t seed = 12345;
    auto nextNoise = [&seed]() -> float
    {
        seed = seed * 1103515245 + 12345;
        return (static_cast<float>(seed % 1000) / 1000.0f - 0.5f) * 0.2f;
    };

    for (int i = 1; i <= 200; ++i)
    {
        float x = static_cast<float>(i) * 0.05f;
        float noise = nextNoise();
        float y = trueIntercept + trueSlope * x + noise;

        auto input = this->MakeInput(1.0f, x);
        auto output = this->MakeOutput(y);
        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(trueIntercept), 0.2f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(trueSlope), 0.2f));
}

TYPED_TEST(RecursiveLeastSquaresTest, TracksTimeVaryingSystem)
{
    typename TestFixture::EstimatorType rls(1000.0f, 0.95f);

    for (int i = 1; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        float y = 1.0f + 2.0f * x;

        auto input = this->MakeInput(1.0f, x);
        auto output = this->MakeOutput(y);
        rls.Update(input, output);
    }

    for (int i = 1; i <= 200; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        float y = 3.0f + 4.0f * x;

        auto input = this->MakeInput(1.0f, x);
        auto output = this->MakeOutput(y);
        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(3.0f), 0.5f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(4.0f), 0.5f));
}

TYPED_TEST(RecursiveLeastSquaresTest, InitializesWithDefaultCovariance)
{
    typename TestFixture::EstimatorType rls(std::nullopt, 1.0f);

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(0.0f)));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(0.0f)));
}

TYPED_TEST(RecursiveLeastSquaresTest, SingleFeatureNoIntercept)
{
    constexpr std::size_t SingleFeature = 1;
    using SingleFeatureRls = estimators::RecursiveLeastSquares<TypeParam, SingleFeature>;

    SingleFeatureRls rls(1000.0f, 1.0f);

    for (int i = 1; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.2f;
        float y = 5.0f * x;

        math::Matrix<TypeParam, 1, 1> input;
        input.at(0, 0) = TypeParam(x);

        math::Matrix<TypeParam, 1, 1> output;
        output.at(0, 0) = TypeParam(y);

        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), TypeParam(5.0f), 0.1f));
}

TYPED_TEST(RecursiveLeastSquaresTest, MultipleFeatures)
{
    constexpr std::size_t ThreeFeatures = 3;
    using MultiFeatureRls = estimators::RecursiveLeastSquares<TypeParam, ThreeFeatures>;

    MultiFeatureRls rls(1000.0f, 0.99f);

    for (int i = 1; i <= 200; ++i)
    {
        float x1 = static_cast<float>(i % 20) * 0.1f + 0.1f;
        float x2 = static_cast<float>((i * 7) % 20) * 0.1f + 0.1f;
        float y = 1.0f + 2.0f * x1 + 3.0f * x2;

        math::Matrix<TypeParam, 3, 1> input;
        input.at(0, 0) = TypeParam(1.0f);
        input.at(1, 0) = TypeParam(x1);
        input.at(2, 0) = TypeParam(x2);

        math::Matrix<TypeParam, 1, 1> output;
        output.at(0, 0) = TypeParam(y);

        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), TypeParam(1.0f), 0.2f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), TypeParam(2.0f), 0.2f));
    EXPECT_TRUE(AreFloatsNear(coef.at(2, 0), TypeParam(3.0f), 0.2f));
}

TYPED_TEST(RecursiveLeastSquaresTest, ForgettingFactorOneNoForgetting)
{
    typename TestFixture::EstimatorType rls(1000.0f, 1.0f);

    for (int i = 1; i <= 50; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        float y = 2.0f + x;

        auto input = this->MakeInput(1.0f, x);
        auto output = this->MakeOutput(y);
        rls.Update(input, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(2.0f), 0.1f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(1.0f), 0.1f));
}

TYPED_TEST(RecursiveLeastSquaresTest, MakeRegressorSetsBiasAndSamples)
{
    typename TestFixture::InputType regressor;
    TestFixture::EstimatorType::MakeRegressor(regressor, TypeParam(3.5f));

    EXPECT_TRUE(AreFloatsNear(regressor.at(0, 0), TypeParam(1.0f)));
    EXPECT_TRUE(AreFloatsNear(regressor.at(1, 0), TypeParam(3.5f)));
}

TYPED_TEST(RecursiveLeastSquaresTest, MakeRegressorWithMultipleFeatures)
{
    constexpr std::size_t ThreeFeatures = 3;
    using MultiFeatureRls = estimators::RecursiveLeastSquares<TypeParam, ThreeFeatures>;
    using InputType = typename MultiFeatureRls::InputMatrix;

    InputType regressor;
    MultiFeatureRls::MakeRegressor(regressor, TypeParam(1.5f), TypeParam(2.5f));

    EXPECT_TRUE(AreFloatsNear(regressor.at(0, 0), TypeParam(1.0f)));
    EXPECT_TRUE(AreFloatsNear(regressor.at(1, 0), TypeParam(1.5f)));
    EXPECT_TRUE(AreFloatsNear(regressor.at(2, 0), TypeParam(2.5f)));
}

TYPED_TEST(RecursiveLeastSquaresTest, MakeRegressorReturnsReference)
{
    typename TestFixture::InputType regressor;
    auto& returned = TestFixture::EstimatorType::MakeRegressor(regressor, TypeParam(1.0f));

    EXPECT_EQ(&returned, &regressor);
}

TYPED_TEST(RecursiveLeastSquaresTest, MakeRegressorWorksWithUpdate)
{
    constexpr float trueIntercept = 2.0f;
    constexpr float trueSlope = 3.0f;

    typename TestFixture::EstimatorType rls(1000.0f, 0.99f);
    typename TestFixture::InputType regressor;
    typename TestFixture::OutputType output;

    for (int i = 1; i <= 100; ++i)
    {
        float x = static_cast<float>(i) * 0.1f;
        float y = trueIntercept + trueSlope * x;

        TestFixture::EstimatorType::MakeRegressor(regressor, TypeParam(x));
        output.at(0, 0) = TypeParam(y);
        rls.Update(regressor, output);
    }

    const auto& coef = rls.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(trueIntercept), 0.1f));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(trueSlope), 0.1f));
}
