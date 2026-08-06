#include "numerical/estimators/offline/LinearRegression.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class LinearRegressionTest
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Samples = 4;
        static constexpr std::size_t Features = 2;

        using EstimatorType = estimators::LinearRegression<T, Samples, Features>;
        using MatrixType = math::Matrix<T, Samples, Features>;
        using TargetType = math::Matrix<T, Samples, 1>;
        using InputType = typename EstimatorType::InputMatrix;

        EstimatorType estimator;

        static T MakeValue(float f)
        {
            return T(std::max(std::min(f, 0.1f), -0.1f));
        }

        MatrixType MakeFeatureMatrix(const std::initializer_list<std::initializer_list<float>>& values)
        {
            MatrixType result;
            std::size_t i = 0;
            for (const auto& row : values)
            {
                std::size_t j = 0;
                for (float value : row)
                {
                    result.at(i, j) = MakeValue(value);
                    ++j;
                }
                ++i;
            }
            return result;
        }

        TargetType MakeTargetVector(const std::initializer_list<float>& values)
        {
            TargetType result;
            std::size_t i = 0;
            for (float value : values)
            {
                result.at(i, 0) = MakeValue(value);
                ++i;
            }
            return result;
        }

        InputType MakeInputVector(const std::initializer_list<float>& values)
        {
            InputType result;
            std::size_t i = 0;
            for (float value : values)
            {
                result.at(i, 0) = MakeValue(value);
                ++i;
            }
            return result;
        }
    };

    using TestTypes = ::testing::Types<float /*, math::Q15, math::Q31*/>;
    TYPED_TEST_SUITE(LinearRegressionTest, TestTypes);
}

TYPED_TEST(LinearRegressionTest, FitRecoversTwoFeatureCoefficients)
{
    auto X = this->MakeFeatureMatrix({ { 0.02f, 0.03f },
        { 0.03f, 0.04f },
        { 0.04f, 0.02f },
        { 0.05f, 0.05f } });

    auto y = this->MakeTargetVector({ 0.02f * 0.05f + 0.03f * 0.03f + 0.01f,
        0.03f * 0.05f + 0.04f * 0.03f + 0.01f,
        0.04f * 0.05f + 0.02f * 0.03f + 0.01f,
        0.05f * 0.05f + 0.05f * 0.03f + 0.01f });

    this->estimator.Fit(X, y);

    const auto& coef = this->estimator.Coefficients();
    EXPECT_NEAR(math::ToFloat(coef.at(0, 0)), math::ToFloat(this->MakeValue(0.01f)), math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(coef.at(1, 0)), math::ToFloat(this->MakeValue(0.05f)), math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(coef.at(2, 0)), math::ToFloat(this->MakeValue(0.03f)), math::Tolerance<float>());
}

TYPED_TEST(LinearRegressionTest, PredictReturnsInterpolatedValue)
{
    auto X = this->MakeFeatureMatrix({ { 0.01f, 0.01f },
        { 0.02f, 0.02f },
        { 0.03f, 0.01f },
        { 0.02f, 0.03f } });

    auto y = this->MakeTargetVector({
        0.01f * 0.02f + 0.01f * 0.01f + 0.01f,
        0.02f * 0.02f + 0.02f * 0.01f + 0.01f,
        0.03f * 0.02f + 0.01f * 0.01f + 0.01f,
        0.02f * 0.02f + 0.03f * 0.01f + 0.01f,
    });

    this->estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.02f, 0.02f });
    auto predicted = this->estimator.Predict(newX);

    EXPECT_NEAR(math::ToFloat(predicted), math::ToFloat(this->MakeValue(0.0106f)), math::Tolerance<float>());
}

TYPED_TEST(LinearRegressionTest, NearZeroFeaturesYieldConstantPrediction)
{
    auto X = this->MakeFeatureMatrix({ { 0.001f, 0.001f },
        { 0.001f, -0.001f },
        { -0.001f, 0.001f },
        { -0.001f, -0.001f } });

    auto y = this->MakeTargetVector({ 0.01f, 0.01f, 0.01f, 0.01f });

    this->estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.001f, 0.001f });
    auto predicted = this->estimator.Predict(newX);

    EXPECT_NEAR(math::ToFloat(predicted), math::ToFloat(this->MakeValue(0.01f)), math::Tolerance<float>());
}

TYPED_TEST(LinearRegressionTest, PredictionStaysWithinTrainingRange)
{
    auto X = this->MakeFeatureMatrix({ { 0.02f, 0.02f },
        { -0.02f, -0.02f },
        { 0.02f, -0.02f },
        { -0.02f, 0.02f } });

    auto y = this->MakeTargetVector({ 0.02f, -0.02f, 0.0f, 0.0f });

    this->estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.02f, 0.02f });
    auto predicted = this->estimator.Predict(newX);

    EXPECT_LE(std::abs(math::ToFloat(predicted)), 0.02f);
}
