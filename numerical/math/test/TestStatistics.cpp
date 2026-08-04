#include "numerical/math/QNumber.hpp"
#include "numerical/math/Statistics.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class StatisticsTest
        : public ::testing::Test
    {
    protected:
        using VectorType = math::Vector<T, 4>;
        using MatrixType = math::Matrix<T, 2, 2>;

        static T MakeValue(float f)
        {
            if constexpr (std::is_same_v<T, float>)
                return f;
            else
                return T{ std::max(std::min(f, 0.9999f), -0.9999f) };
        }

        VectorType MakeVector(float a, float b, float c, float d)
        {
            return VectorType{
                { MakeValue(a) },
                { MakeValue(b) },
                { MakeValue(c) },
                { MakeValue(d) }
            };
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(StatisticsTest, TestTypes);

    class StatisticsFloatTest
        : public ::testing::Test
    {};
}

TYPED_TEST(StatisticsTest, MeanOfUniformlySpacedValues)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::Mean(data);

    EXPECT_NEAR(math::ToFloat(result), 0.05f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, PopulationVarianceOfUniformlySpacedValues)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::Variance(data, false);

    EXPECT_NEAR(math::ToFloat(result), 0.0005f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, SampleVarianceOfUniformlySpacedValues)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::Variance(data, true);

    EXPECT_NEAR(math::ToFloat(result), 0.000667f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, PopulationStandardDeviationOfUniformlySpacedValues)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::StandardDeviation(data, false);

    EXPECT_NEAR(math::ToFloat(result), 0.02236f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, SampleStandardDeviationOfUniformlySpacedValues)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::StandardDeviation(data, true);

    EXPECT_NEAR(math::ToFloat(result), 0.02582f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, MeanSquaredErrorOfKnownPredictions)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::MeanSquaredError(actual, predicted);

    EXPECT_NEAR(math::ToFloat(result), 0.01f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, RootMeanSquaredErrorOfKnownPredictions)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::RootMeanSquaredError(actual, predicted);

    EXPECT_NEAR(math::ToFloat(result), 0.1f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, MeanAbsoluteErrorOfKnownPredictions)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::MeanAbsoluteError(actual, predicted);

    EXPECT_NEAR(math::ToFloat(result), 0.1f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, RSquaredScoreOfNearPerfectPredictions)
{
    auto actual = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);
    auto predicted = this->MakeVector(0.03f, 0.03f, 0.07f, 0.07f);

    auto result = math::RSquaredScore(actual, predicted);

    EXPECT_NEAR(math::ToFloat(result), 0.8f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, AutoCorrelationLagZeroIsUnity)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::AutoCorrelation(data, 2);

    EXPECT_NEAR(math::ToFloat(result.at(0, 0)), 0.9999f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, AutoCorrelationLagOneValue)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::AutoCorrelation(data, 2);

    EXPECT_NEAR(math::ToFloat(result.at(1, 0)), 0.3333f, math::Tolerance<float>());
}

TYPED_TEST(StatisticsTest, AutoCorrelationLagTwoValue)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto result = math::AutoCorrelation(data, 2);

    EXPECT_NEAR(math::ToFloat(result.at(2, 0)), -0.6f, math::Tolerance<float>());
}

TEST_F(StatisticsFloatTest, ZScoreNormalizesSymmetricData)
{
    auto data = math::Matrix<float, 2, 2>{ 0.45f, 0.5f, 0.5f, 0.55f };

    auto result = math::ZScore(data);

    EXPECT_NEAR(result.at(0, 0), -1.4142f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 1), 1.4142f, math::Tolerance<float>());
}
