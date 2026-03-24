#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T, std::size_t Size>
    bool AreVectorsNear(const math::Vector<T, Size>& a,
        const math::Vector<T, Size>& b,
        float epsilon)
    {
        for (std::size_t i = 0; i < Size; ++i)
            if (std::abs(math::ToFloat(a.at(i, 0)) - math::ToFloat(b.at(i, 0))) >= epsilon)
                return false;

        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    bool AreMatricesNear(const math::Matrix<T, Rows, Cols>& a,
        const math::Matrix<T, Rows, Cols>& b,
        float epsilon)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                if (std::abs(math::ToFloat(a.at(i, j)) - math::ToFloat(b.at(i, j))) >= epsilon)
                    return false;

        return true;
    }

    template<typename T>
    class KalmanFilterTest
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t StateSize = 2;
        static constexpr std::size_t MeasurementSize = 1;

        using FilterType = filters::KalmanFilter<T, StateSize, MeasurementSize>;
        using StateVector = typename FilterType::StateVector;
        using StateMatrix = typename FilterType::StateMatrix;
        using MeasurementVector = typename FilterType::MeasurementVector;
        using MeasurementMatrix = typename FilterType::MeasurementMatrix;
        using MeasurementCovariance = typename FilterType::MeasurementCovariance;

        float tolerance = math::Tolerance<T>();

        static T MakeValue(float f)
        {
            return T(f);
        }

        StateVector MakeStateVector(float x, float v)
        {
            return StateVector{
                { MakeValue(x) },
                { MakeValue(v) }
            };
        }

        StateMatrix MakeStateMatrix(
            float a11, float a12,
            float a21, float a22)
        {
            return StateMatrix{
                { MakeValue(a11), MakeValue(a12) },
                { MakeValue(a21), MakeValue(a22) }
            };
        }

        MeasurementMatrix MakeMeasurementMatrix(float h1, float h2)
        {
            return MeasurementMatrix{
                { MakeValue(h1), MakeValue(h2) }
            };
        }

        MeasurementCovariance MakeMeasurementCovariance(float r)
        {
            return MeasurementCovariance{
                { MakeValue(r) }
            };
        }

        MeasurementVector MakeMeasurement(float z)
        {
            return MeasurementVector{ { MakeValue(z) } };
        }
    };

    using TestTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(KalmanFilterTest, TestTypes);
}

TYPED_TEST(KalmanFilterTest, DefaultInitialization)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f,
        0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    EXPECT_TRUE(AreVectorsNear(filter.GetState(), initialState, this->tolerance));
    EXPECT_TRUE(AreMatricesNear(filter.GetCovariance(), initialCovariance, this->tolerance));
}

TYPED_TEST(KalmanFilterTest, PredictConstantVelocity)
{
    auto initialState = this->MakeStateVector(0.0f, 0.1f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f,
        0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeStateMatrix(
        1.0f, dt,
        0.0f, 1.0f));

    filter.Predict();

    // x_new = [0 + 0.1*0.1, 0.1] = [0.01, 0.1]
    auto expectedState = this->MakeStateVector(0.01f, 0.1f);
    EXPECT_TRUE(AreVectorsNear(filter.GetState(), expectedState, this->tolerance));
}

TYPED_TEST(KalmanFilterTest, UpdateCorrectlyFusesInformation)
{
    // Position-velocity system, measure position only
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.5f, 0.0f,
        0.0f, 0.5f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(1.0f, 0.0f));
    filter.SetMeasurementNoise(this->MakeMeasurementCovariance(0.5f));

    // Measurement of position = 0.5
    filter.Update(this->MakeMeasurement(0.5f));

    auto updatedState = filter.GetState();
    // With equal P and R, the gain should be ~0.5, so state should be ~0.25
    float posEstimate = math::ToFloat(updatedState.at(0, 0));
    EXPECT_GT(posEstimate, 0.0f);
    EXPECT_LT(posEstimate, 0.5f);

    // Covariance should decrease after update
    auto updatedP = filter.GetCovariance();
    EXPECT_LT(math::ToFloat(updatedP.at(0, 0)), 0.5f);
}

TYPED_TEST(KalmanFilterTest, FullPredictUpdateCycle)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.5f, 0.0f,
        0.0f, 0.5f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeStateMatrix(
        1.0f, dt,
        0.0f, 1.0f));
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(1.0f, 0.0f));
    filter.SetMeasurementNoise(this->MakeMeasurementCovariance(0.5f));
    filter.SetProcessNoise(this->MakeStateMatrix(
        0.01f, 0.0f,
        0.0f, 0.01f));

    // Simulate 10 steps - estimator should track object moving at velocity ~1.0
    float truePos = 0.0f;
    float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * dt;
        float measurement = truePos;

        filter.Predict();
        filter.Update(this->MakeMeasurement(measurement));
    }

    auto finalState = filter.GetState();
    float posEstimate = math::ToFloat(finalState.at(0, 0));
    float velEstimate = math::ToFloat(finalState.at(1, 0));

    EXPECT_NEAR(posEstimate, truePos, 0.2f);
    EXPECT_NEAR(velEstimate, trueVel, 0.4f);
}

TYPED_TEST(KalmanFilterTest, CovarianceEvolution)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.5f, 0.0f,
        0.0f, 0.5f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeStateMatrix(
        1.0f, dt,
        0.0f, 1.0f));
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(1.0f, 0.0f));
    filter.SetMeasurementNoise(this->MakeMeasurementCovariance(0.5f));
    filter.SetProcessNoise(this->MakeStateMatrix(
        0.01f, 0.0f,
        0.0f, 0.01f));

    filter.Predict();
    auto predictedCovariance = filter.GetCovariance();
    EXPECT_GT(math::ToFloat(predictedCovariance.at(0, 0)), math::ToFloat(initialCovariance.at(0, 0)));

    filter.Update(this->MakeMeasurement(0.1f));
    auto updatedCovariance = filter.GetCovariance();
    EXPECT_LT(math::ToFloat(updatedCovariance.at(0, 0)), math::ToFloat(predictedCovariance.at(0, 0)));
}

TYPED_TEST(KalmanFilterTest, PredictOnlyGrowsCovariance)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        0.1f, 0.0f,
        0.0f, 0.1f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    filter.SetStateTransition(this->MakeStateMatrix(
        1.0f, 0.1f,
        0.0f, 1.0f));
    filter.SetProcessNoise(this->MakeStateMatrix(
        0.01f, 0.0f,
        0.0f, 0.01f));

    for (int i = 0; i < 5; ++i)
        filter.Predict();

    auto P = filter.GetCovariance();
    EXPECT_GT(math::ToFloat(P.at(0, 0)), 0.1f);
    EXPECT_GT(math::ToFloat(P.at(1, 1)), 0.1f);
}

TYPED_TEST(KalmanFilterTest, ThreeStateConstantAcceleration)
{
    using T = TypeParam;
    using Filter3 = filters::KalmanFilter<T, 3, 1, 0>;
    using StateVec3 = math::Vector<T, 3>;
    using StateMat3 = math::SquareMatrix<T, 3>;
    using MeasMat = math::Matrix<T, 1, 3>;
    using MeasCov = math::SquareMatrix<T, 1>;
    using MeasVec = math::Vector<T, 1>;

    float dt = 0.1f;

    auto initialState = StateVec3{ { T(0.0f) }, { T(0.0f) }, { T(0.0f) } };
    auto initialP = StateMat3{
        { T(1.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(1.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(1.0f) }
    };

    Filter3 filter(initialState, initialP);

    filter.SetStateTransition(StateMat3{
        { T(1.0f), T(dt), T(0.5f * dt * dt) },
        { T(0.0f), T(1.0f), T(dt) },
        { T(0.0f), T(0.0f), T(1.0f) } });
    filter.SetMeasurementMatrix(MeasMat{ { T(1.0f), T(0.0f), T(0.0f) } });
    filter.SetMeasurementNoise(MeasCov{ { T(0.1f) } });
    filter.SetProcessNoise(StateMat3{
        { T(0.01f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.01f), T(0.0f) },
        { T(0.0f), T(0.0f), T(0.01f) } });

    float trueAcc = 1.0f;
    float trueVel = 0.0f;
    float truePos = 0.0f;

    for (int i = 0; i < 20; ++i)
    {
        trueVel += trueAcc * dt;
        truePos += trueVel * dt;

        filter.Predict();
        filter.Update(MeasVec{ { T(truePos) } });
    }

    auto finalState = filter.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.3f);
}

TYPED_TEST(KalmanFilterTest, FourStateTwoMeasurement)
{
    using T = TypeParam;
    using Filter4 = filters::KalmanFilter<T, 4, 2, 0>;
    using StateVec4 = math::Vector<T, 4>;
    using StateMat4 = math::SquareMatrix<T, 4>;
    using MeasMat = math::Matrix<T, 2, 4>;
    using MeasCov = math::SquareMatrix<T, 2>;
    using MeasVec = math::Vector<T, 2>;

    float dt = 0.1f;

    auto initialState = StateVec4{ { T(0.0f) }, { T(0.0f) }, { T(0.0f) }, { T(0.0f) } };
    auto initialP = StateMat4{
        { T(1.0f), T(0.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(1.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(1.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(0.0f), T(1.0f) }
    };

    Filter4 filter(initialState, initialP);

    // State: [x, vx, y, vy], constant velocity model
    filter.SetStateTransition(StateMat4{
        { T(1.0f), T(dt), T(0.0f), T(0.0f) },
        { T(0.0f), T(1.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(1.0f), T(dt) },
        { T(0.0f), T(0.0f), T(0.0f), T(1.0f) } });

    // Measure x and y positions
    filter.SetMeasurementMatrix(MeasMat{
        { T(1.0f), T(0.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(1.0f), T(0.0f) } });

    filter.SetMeasurementNoise(MeasCov{
        { T(0.1f), T(0.0f) },
        { T(0.0f), T(0.1f) } });
    filter.SetProcessNoise(StateMat4{
        { T(0.01f), T(0.0f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.01f), T(0.0f), T(0.0f) },
        { T(0.0f), T(0.0f), T(0.01f), T(0.0f) },
        { T(0.0f), T(0.0f), T(0.0f), T(0.01f) } });

    float trueX = 0.0f, trueVx = 0.5f;
    float trueY = 0.0f, trueVy = 0.3f;

    for (int i = 0; i < 20; ++i)
    {
        trueX += trueVx * dt;
        trueY += trueVy * dt;

        filter.Predict();
        filter.Update(MeasVec{ { T(trueX) }, { T(trueY) } });
    }

    auto finalState = filter.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), trueX, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(2, 0)), trueY, 0.2f);
}

TYPED_TEST(KalmanFilterTest, WithControlInput)
{
    using T = TypeParam;
    using FilterCtrl = filters::KalmanFilter<T, 2, 1, 1>;
    using StateVec = math::Vector<T, 2>;
    using StateMat = math::SquareMatrix<T, 2>;
    using MeasMat = math::Matrix<T, 1, 2>;
    using MeasCov = math::SquareMatrix<T, 1>;
    using MeasVec = math::Vector<T, 1>;
    using CtrlMat = math::Matrix<T, 2, 1>;
    using CtrlVec = math::Vector<T, 1>;

    float dt = 0.1f;

    auto initialState = StateVec{ { T(0.0f) }, { T(0.0f) } };
    auto initialP = StateMat{
        { T(0.5f), T(0.0f) },
        { T(0.0f), T(0.5f) }
    };

    FilterCtrl filter(initialState, initialP);

    filter.SetStateTransition(StateMat{
        { T(1.0f), T(dt) },
        { T(0.0f), T(1.0f) } });
    filter.SetControlInputMatrix(CtrlMat{
        { T(0.5f * dt * dt) },
        { T(dt) } });
    filter.SetMeasurementMatrix(MeasMat{ { T(1.0f), T(0.0f) } });
    filter.SetMeasurementNoise(MeasCov{ { T(0.1f) } });
    filter.SetProcessNoise(StateMat{
        { T(0.01f), T(0.0f) },
        { T(0.0f), T(0.01f) } });

    CtrlVec controlInput{ { T(1.0f) } };
    float truePos = 0.0f, trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * dt;
        truePos += trueVel * dt;

        filter.Predict(controlInput);
        filter.Update(MeasVec{ { T(truePos) } });
    }

    auto finalState = filter.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.4f);
}

TYPED_TEST(KalmanFilterTest, JosephFormPreservesPositiveDefiniteness)
{
    auto initialState = this->MakeStateVector(0.0f, 0.0f);
    auto initialCovariance = this->MakeStateMatrix(
        1000.0f, 0.0f,
        0.0f, 1000.0f);

    typename TestFixture::FilterType filter(initialState, initialCovariance);

    float dt = 0.1f;
    filter.SetStateTransition(this->MakeStateMatrix(
        1.0f, dt,
        0.0f, 1.0f));
    filter.SetMeasurementMatrix(this->MakeMeasurementMatrix(1.0f, 0.0f));
    filter.SetMeasurementNoise(this->MakeMeasurementCovariance(0.001f));
    filter.SetProcessNoise(this->MakeStateMatrix(
        0.001f, 0.0f,
        0.0f, 0.001f));

    for (int i = 0; i < 50; ++i)
    {
        filter.Predict();
        filter.Update(this->MakeMeasurement(1.0f));

        auto P = filter.GetCovariance();
        EXPECT_GE(math::ToFloat(P.at(0, 0)), 0.0f);
        EXPECT_GE(math::ToFloat(P.at(1, 1)), 0.0f);

        float det = math::ToFloat(P.at(0, 0)) * math::ToFloat(P.at(1, 1)) -
                    math::ToFloat(P.at(0, 1)) * math::ToFloat(P.at(1, 0));
        EXPECT_GE(det, 0.0f);
    }
}
