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
            return (std::abs(math::ToFloat(a.at(i, 0)) - math::ToFloat(b.at(i, 0))) < epsilon);

        return true;
    }

    template<typename T>
    bool AreMatricesNear(const math::Matrix<T, 3, 3>& a,
        const math::Matrix<T, 3, 3>& b,
        float epsilon = 1e-4f)
    {
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                return (std::abs(math::ToFloat(a.at(i, j)) - math::ToFloat(b.at(i, j))) < epsilon);

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
                0.1f, dt, dt2_2,
                0.0f, 0.1f, dt,
                0.0f, 0.0f, 0.1f);
        }
    };

    using TestTypes = ::testing::Types<float /*, math::Q15, math::Q31*/>;
    TYPED_TEST_SUITE(KalmanFilterTest, TestTypes);
}

TYPED_TEST(KalmanFilterTest, DefaultInitialization)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    EXPECT_TRUE(AreVectorsNear(filter.GetState(), initialState));
    EXPECT_TRUE(AreMatricesNear(filter.GetCovariance(), initialCovariance));
}

TYPED_TEST(KalmanFilterTest, PredictConstantAcceleration)
{
    auto initialState = this->MakeStateVector(0.0f, 0.01f, 0.005f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeConstantAccelerationModel(dt));

    filter.Predict();

    auto expectedState = this->MakeStateVector(
        0.001025f,
        0.0105f,
        0.005f);

    EXPECT_TRUE(AreVectorsNear(filter.GetState(), expectedState));
}

TYPED_TEST(KalmanFilterTest, UpdateWithPositionMeasurement)
{
    auto initialState = this->MakeStateVector(0.0f, 0.1f, 0.05f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    auto measurementMatrix = this->MakeMeasurementMatrix(0.1f, 0.0f, 0.0f);
    filter.SetMeasurementMatrix(measurementMatrix);

    auto measurement = this->MakeMeasurement(0.1f);
    filter.Update(measurement);

    auto updatedState = filter.GetState();
    EXPECT_GT(math::ToFloat(updatedState.at(0, 0)), math::ToFloat(initialState.at(0, 0)));
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
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(0.1f, 0.0f, 0.0f));

    auto processNoise = this->MakeStateMatrix(
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f);
    filter.SetProcessNoise(processNoise);

    std::vector<float> measurements = { 0.01f, 0.02f, 0.035f, 0.05f };

    for (float measurement : measurements)
    {
        filter.Predict();
        filter.Update(this->MakeMeasurement(measurement));

        auto state = filter.GetState();
        EXPECT_LE(math::ToFloat(state.at(0, 0)), measurement);
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
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(0.1f, 0.0f, 0.0f));

    filter.Predict();
    auto predictedCovariance = filter.GetCovariance();
    EXPECT_GT(math::ToFloat(predictedCovariance.at(0, 0)), math::ToFloat(initialCovariance.at(0, 0)));

    filter.Update(this->MakeMeasurement(0.1f));
    auto updatedCovariance = filter.GetCovariance();
    EXPECT_LT(math::ToFloat(updatedCovariance.at(0, 0)), math::ToFloat(predictedCovariance.at(0, 0)));
}
