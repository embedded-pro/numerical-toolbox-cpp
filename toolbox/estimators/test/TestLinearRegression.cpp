#include "toolbox/estimators/LinearRegression.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    bool AreFloatsNear(T a, T b, float epsilon = 1e-4f)
    {
        if constexpr (std::is_same_v<T, float>)
            return std::abs(a - b) < epsilon;
        else
            return std::abs(a.ToFloat() - b.ToFloat()) < epsilon;
    }

    template<typename T>
    class LinearRegressionTest
        : public ::testing::Test
    {
    protected:
        static constexpr size_t Samples = 4;
        static constexpr size_t Features = 2;

        using EstimatorType = estimators::LinearRegression<T, Samples, Features>;
        using MatrixType = math::Matrix<T, Samples, Features>;
        using TargetType = math::Matrix<T, Samples, 1>;
        using InputType = typename EstimatorType::InputMatrix;

        static T MakeValue(float f)
        {
            return T(std::max(std::min(f, 0.1f), -0.1f));
        }

        MatrixType MakeFeatureMatrix(const std::initializer_list<std::initializer_list<float>>& values)
        {
            MatrixType result;
            size_t i = 0;
            for (const auto& row : values)
            {
                size_t j = 0;
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
            size_t i = 0;
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
            size_t i = 0;
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

TYPED_TEST(LinearRegressionTest, SimpleLinearFit)
{
    typename TestFixture::EstimatorType estimator;

    auto X = this->MakeFeatureMatrix({ { 0.02f, 0.03f },
        { 0.03f, 0.04f },
        { 0.04f, 0.02f },
        { 0.05f, 0.05f } });

    auto y = this->MakeTargetVector({ 0.02f * 0.05f + 0.03f * 0.03f + 0.01f,
        0.03f * 0.05f + 0.04f * 0.03f + 0.01f,
        0.04f * 0.05f + 0.02f * 0.03f + 0.01f,
        0.05f * 0.05f + 0.05f * 0.03f + 0.01f });

    estimator.Fit(X, y);

    const auto& coef = estimator.Coefficients();
    EXPECT_TRUE(AreFloatsNear(coef.at(0, 0), this->MakeValue(0.01f)));
    EXPECT_TRUE(AreFloatsNear(coef.at(1, 0), this->MakeValue(0.05f)));
    EXPECT_TRUE(AreFloatsNear(coef.at(2, 0), this->MakeValue(0.03f)));
}

TYPED_TEST(LinearRegressionTest, PredictNewValues)
{
    typename TestFixture::EstimatorType estimator;

    auto X = this->MakeFeatureMatrix({ { 0.01f, 0.01f },
        { 0.02f, 0.02f },
        { 0.03f, 0.01f },
        { 0.02f, 0.03f } });

    auto y = this->MakeTargetVector({
        0.01f * 0.02f + 0.01f * 0.01f + 0.01f, // 0.0103
        0.02f * 0.02f + 0.02f * 0.01f + 0.01f, // 0.0106
        0.03f * 0.02f + 0.01f * 0.01f + 0.01f, // 0.0107
        0.02f * 0.02f + 0.03f * 0.01f + 0.01f  // 0.0107
    });

    estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.02f, 0.02f });
    auto predicted = estimator.Predict(newX);

    EXPECT_TRUE(AreFloatsNear(predicted, this->MakeValue(0.0106f)));
}

TYPED_TEST(LinearRegressionTest, NearZeroFeatures)
{
    typename TestFixture::EstimatorType estimator;

    auto X = this->MakeFeatureMatrix({ { 0.001f, 0.001f },
        { 0.001f, -0.001f },
        { -0.001f, 0.001f },
        { -0.001f, -0.001f } });

    auto y = this->MakeTargetVector({ 0.01f, 0.01f, 0.01f, 0.01f });

    estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.001f, 0.001f });
    auto predicted = estimator.Predict(newX);

    EXPECT_TRUE(AreFloatsNear(predicted, this->MakeValue(0.01f)));
}

TYPED_TEST(LinearRegressionTest, RangeLimits)
{
    typename TestFixture::EstimatorType estimator;

    auto X = this->MakeFeatureMatrix({ { 0.02f, 0.02f },
        { -0.02f, -0.02f },
        { 0.02f, -0.02f },
        { -0.02f, 0.02f } });

    auto y = this->MakeTargetVector({ 0.02f, -0.02f, 0.0f, 0.0f });

    estimator.Fit(X, y);

    auto newX = this->MakeInputVector({ 0.02f, 0.02f });
    auto predicted = estimator.Predict(newX);

    EXPECT_LE(std::abs(math::ToFloat(predicted)), 0.02f);
}
