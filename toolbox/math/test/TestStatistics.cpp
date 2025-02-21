#include "math/QNumber.hpp"
#include "math/Statistics.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    bool AreValuesNear(T a, T b, float epsilon = 1e-4f)
    {
        printf("\noriginal: %f, result: %f\n", math::ToFloat(a), math::ToFloat(b));
        return (std::abs(math::ToFloat(a) - math::ToFloat(b)) < epsilon);
    }

    template<typename T>
    class StatisticsTest
        : public ::testing::Test
    {
    protected:
        using MatrixType = math::Matrix<T, 2, 2>;
        using VectorType = math::Vector<T, 4>;

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

        MatrixType MakeMatrix(float a11, float a12, float a21, float a22)
        {
            return MatrixType{
                { MakeValue(a11), MakeValue(a12) },
                { MakeValue(a21), MakeValue(a22) }
            };
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(StatisticsTest, TestTypes);
}

TYPED_TEST(StatisticsTest, Mean)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);
    auto result = math::Mean(data);
    EXPECT_TRUE(AreValuesNear(result, this->MakeValue(0.05f)));
}

TYPED_TEST(StatisticsTest, Variance)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto resultPopulation = math::Variance(data, false);
    EXPECT_TRUE(AreValuesNear(resultPopulation, this->MakeValue(0.0005f)));

    auto resultSample = math::Variance(data, true);
    EXPECT_TRUE(AreValuesNear(resultSample, this->MakeValue(0.000667f)));
}

TYPED_TEST(StatisticsTest, StandardDeviation)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);

    auto resultPopulation = math::StandardDeviation(data, false);
    EXPECT_TRUE(AreValuesNear(resultPopulation, this->MakeValue(0.02236f), 0.001f));

    auto resultSample = math::StandardDeviation(data, true);
    EXPECT_TRUE(AreValuesNear(resultSample, this->MakeValue(0.02582f), 0.001f));
}

TYPED_TEST(StatisticsTest, MeanSquaredError)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::MeanSquaredError(actual, predicted);
    EXPECT_TRUE(AreValuesNear(result, this->MakeValue(0.01f)));
}

TYPED_TEST(StatisticsTest, RootMeanSquaredError)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::RootMeanSquaredError(actual, predicted);
    EXPECT_TRUE(AreValuesNear(result, this->MakeValue(0.1f)));
}

TYPED_TEST(StatisticsTest, MeanAbsoluteError)
{
    auto actual = this->MakeVector(0.2f, 0.4f, 0.6f, 0.8f);
    auto predicted = this->MakeVector(0.3f, 0.3f, 0.7f, 0.7f);

    auto result = math::MeanAbsoluteError(actual, predicted);
    EXPECT_TRUE(AreValuesNear(result, this->MakeValue(0.1f)));
}

TYPED_TEST(StatisticsTest, RSquaredScore)
{
    auto actual = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);
    auto predicted = this->MakeVector(0.03f, 0.03f, 0.07f, 0.07f);

    auto result = math::RSquaredScore(actual, predicted);
    EXPECT_TRUE(AreValuesNear(result, this->MakeValue(0.8f), 0.001f));
}

TYPED_TEST(StatisticsTest, AutoCorrelation)
{
    auto data = this->MakeVector(0.02f, 0.04f, 0.06f, 0.08f);
    auto result = math::AutoCorrelation(data, 2);

    auto expected = this->MakeVector(0.9999f, 0.3333f, -0.6f, 0.0f);
    for (size_t i = 0; i < 3; ++i)
        EXPECT_TRUE(AreValuesNear(result.at(i, 0), expected.at(i, 0), 0.001f));
}

TEST(StatisticsTest, ZScore)
{
    auto data = math::Matrix<float, 2, 2>(0.45f, 0.5f, 0.5f, 0.55f);
    auto result = math::ZScore(data);

    auto expected = math::Matrix<float, 2, 2>(-1.4142f, 0.0f, 0.0f, 1.4142f);
    for (size_t i = 0; i < 2; ++i)
        for (size_t j = 0; j < 2; ++j)
            EXPECT_TRUE(AreValuesNear(result.at(i, j), expected.at(i, j)));
}
