#include "filters/active/KalmanFilter.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    bool AreVectorsNear(const math::Vector<T, 3>& a,
        const math::Vector<T, 3>& b,
        float epsilon = 1e-4f)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            if constexpr (std::is_same_v<T, float>)
                return (std::abs(a.at(i, 0) - b.at(i, 0)) < epsilon);
            else
                return (std::abs(a.at(i, 0).ToFloat() - b.at(i, 0).ToFloat()) < epsilon);
        }
        return true;
    }

    template<typename T>
    bool AreMatricesNear(const math::Matrix<T, 3, 3>& a,
        const math::Matrix<T, 3, 3>& b,
        float epsilon = 1e-4f)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                if constexpr (std::is_same_v<T, float>)
                    return (std::abs(a.at(i, j) - b.at(i, j)) < epsilon);
                else
                    return (std::abs(a.at(i, j).ToFloat() - b.at(i, j).ToFloat()) < epsilon);
            }
        }
        return true;
    }

    template<typename T>
    class KalmanFilterTest
        : public ::testing::Test
    {
    protected:
        static constexpr size_t StateSize = 3;
        static constexpr size_t MeasurementSize = 1;

        using FilterType = filters::KalmanFilter<T, StateSize, MeasurementSize>;
        using StateVector = typename FilterType::StateVector;
        using StateMatrix = typename FilterType::StateMatrix;
        using MeasurementVector = typename FilterType::MeasurementVector;
        using MeasurementMatrix = typename FilterType::MeasurementMatrix;

        static T MakeValue(float f)
        {
            f = std::max(std::min(f, 0.9999f), -0.9999f);
            if constexpr (std::is_same_v<T, float>)
                return f;
            else
                return T(f);
        }

        StateVector MakeStateVector(float x, float v, float a)
        {
            return StateVector{
                { MakeValue(x) },
                { MakeValue(v) },
                { MakeValue(a) }
            };
        }

        StateMatrix MakeStateMatrix(
            float a11, float a12, float a13,
            float a21, float a22, float a23,
            float a31, float a32, float a33)
        {
            return StateMatrix{
                { MakeValue(a11), MakeValue(a12), MakeValue(a13) },
                { MakeValue(a21), MakeValue(a22), MakeValue(a23) },
                { MakeValue(a31), MakeValue(a32), MakeValue(a33) }
            };
        }

        MeasurementMatrix MakeMeasurementMatrix(float h1, float h2, float h3)
        {
            return MeasurementMatrix{
                { MakeValue(h1), MakeValue(h2), MakeValue(h3) }
            };
        }

        MeasurementVector MakeMeasurement(float z)
        {
            return MeasurementVector{ { MakeValue(z) } };
        }

        StateMatrix MakeConstantAccelerationModel(float dt)
        {
            float dt2_2 = dt * dt * 0.5f;
            return MakeStateMatrix(
                0.9999f, dt, dt2_2,
                0.0f, 0.9999f, dt,
                0.0f, 0.0f, 0.9999f);
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(KalmanFilterTest, TestTypes);
}

TYPED_TEST(KalmanFilterTest, DefaultInitialization)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.9999f, 0.0f, 0.0f,
        0.0f, 0.9999f, 0.0f,
        0.0f, 0.0f, 0.9999f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    EXPECT_TRUE(AreVectorsNear(filter.GetState(), initialState));
    EXPECT_TRUE(AreMatricesNear(filter.GetCovariance(), initialCovariance));
}

TYPED_TEST(KalmanFilterTest, PredictConstantAcceleration)
{
    auto initialState = this->MakeStateVector(0.0f, 0.1f, 0.05f);
    auto initialCovariance = this->MakeStateMatrix(
        0.9999f, 0.0f, 0.0f,
        0.0f, 0.9999f, 0.0f,
        0.0f, 0.0f, 0.9999f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeConstantAccelerationModel(dt));

    filter.Predict();

    auto expectedState = this->MakeStateVector(
        0.01025f,
        0.105f,
        0.05f);

    EXPECT_TRUE(AreVectorsNear(filter.GetState(), expectedState));
}

TYPED_TEST(KalmanFilterTest, UpdateWithPositionMeasurement)
{
    auto initialState = this->MakeStateVector(0.0f, 0.1f, 0.05f);
    auto initialCovariance = this->MakeStateMatrix(
        0.9999f, 0.0f, 0.0f,
        0.0f, 0.9999f, 0.0f,
        0.0f, 0.0f, 0.9999f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    auto measurementMatrix = this->MakeMeasurementMatrix(0.9999f, 0.0f, 0.0f);
    filter.SetMeasurementMatrix(measurementMatrix);

    auto measurement = this->MakeMeasurement(0.1f);
    filter.Update(measurement);

    auto updatedState = filter.GetState();
    if constexpr (std::is_same_v<TypeParam, float>)
        EXPECT_GT(updatedState.at(0, 0), initialState.at(0, 0));
    else
        EXPECT_GT(updatedState.at(0, 0).ToFloat(), initialState.at(0, 0).ToFloat());
}

TYPED_TEST(KalmanFilterTest, FullTrajectory)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeConstantAccelerationModel(dt));
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(0.9999f, 0.0f, 0.0f));

    auto processNoise = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);
    filter.SetProcessNoise(processNoise);

    std::vector<float> measurements = { 0.1f, 0.2f, 0.35f, 0.5f };

    for (float measurement : measurements)
    {
        filter.Predict();
        filter.Update(this->MakeMeasurement(measurement));

        auto state = filter.GetState();
        if constexpr (std::is_same_v<TypeParam, float>)
            EXPECT_LE(state.at(0, 0), measurement);
        else
            EXPECT_LE(state.at(0, 0).ToFloat(), measurement);
    }
}

TYPED_TEST(KalmanFilterTest, CovarianceEvolution)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeConstantAccelerationModel(dt));
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(0.9999f, 0.0f, 0.0f));

    filter.Predict();
    auto predictedCovariance = filter.GetCovariance();
    if constexpr (std::is_same_v<TypeParam, float>)
        EXPECT_GT(predictedCovariance.at(0, 0), initialCovariance.at(0, 0));
    else
        EXPECT_GT(predictedCovariance.at(0, 0).ToFloat(), initialCovariance.at(0, 0).ToFloat());

    filter.Update(this->MakeMeasurement(0.1f));
    auto updatedCovariance = filter.GetCovariance();
    if constexpr (std::is_same_v<TypeParam, float>)
        EXPECT_LT(updatedCovariance.at(0, 0), predictedCovariance.at(0, 0));
    else
        EXPECT_LT(updatedCovariance.at(0, 0).ToFloat(), predictedCovariance.at(0, 0).ToFloat());
}
