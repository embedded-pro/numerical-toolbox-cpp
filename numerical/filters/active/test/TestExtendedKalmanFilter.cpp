#include "numerical/filters/active/ExtendedKalmanFilter.hpp"
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

    // Linear state transition: x_new = F * x (constant velocity model)
    using StateVec2 = math::Vector<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using StateMat2 = math::SquareMatrix<float, 2>;
    using MeasMat2x1 = math::Matrix<float, 1, 2>;

    constexpr float dt = 0.1f;

    StateVec2 LinearStateTransition(const StateVec2& x)
    {
        return StateVec2{
            { float(x.at(0, 0) + dt * x.at(1, 0)) },
            { x.at(1, 0) }
        };
    }

    StateMat2 LinearStateJacobian(const StateVec2& /*x*/)
    {
        return StateMat2{
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
    }

    MeasVec1 LinearMeasurement(const StateVec2& x)
    {
        return MeasVec1{ { x.at(0, 0) } };
    }

    MeasMat2x1 LinearMeasurementJacobian(const StateVec2& /*x*/)
    {
        return MeasMat2x1{ { 1.0f, 0.0f } };
    }

    // Nonlinear state transition: position += velocity*dt, velocity += -sin(position)*dt
    StateVec2 NonlinearStateTransition(const StateVec2& x)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * dt },
            { vel - std::sin(pos) * dt }
        };
    }

    StateMat2 NonlinearStateJacobian(const StateVec2& x)
    {
        float pos = x.at(0, 0);
        return StateMat2{
            { 1.0f, dt },
            { -std::cos(pos) * dt, 1.0f }
        };
    }

    // State transition with control input
    using ControlVec1 = math::Vector<float, 1>;

    StateVec2 StateTransitionWithControl(const StateVec2& x, const ControlVec1& u)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * dt },
            { vel + u.at(0, 0) * dt }
        };
    }

    StateMat2 StateJacobianWithControl(const StateVec2& /*x*/, const ControlVec1& /*u*/)
    {
        return StateMat2{
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
    }

    template<typename T>
    class ExtendedKalmanFilterTest
        : public ::testing::Test
    {
    protected:
        float tolerance = math::Tolerance<T>();

        using EkfType = filters::ExtendedKalmanFilter<T, 2, 1, 0>;
        using EkfWithControlType = filters::ExtendedKalmanFilter<T, 2, 1, 1>;
        using StateVector = typename EkfType::StateVector;
        using StateMatrix = typename EkfType::StateMatrix;
        using MeasurementVector = typename EkfType::MeasurementVector;
        using MeasurementCovariance = typename EkfType::MeasurementCovariance;
    };

    using TestTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(ExtendedKalmanFilterTest, TestTypes);
}

TYPED_TEST(ExtendedKalmanFilterTest, LinearSystemMatchesKalmanFilter)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    typename TestFixture::EkfType ekf(initialState, initialP,
        LinearStateTransition, LinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);

    ekf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });
    ekf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    float truePos = 0.0f;
    float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * dt;
        ekf.Predict();
        ekf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ekf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.4f);
}

TYPED_TEST(ExtendedKalmanFilterTest, NonlinearPredictionUpdatesState)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.1f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    typename TestFixture::EkfType ekf(initialState, initialP,
        NonlinearStateTransition, NonlinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);

    ekf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });

    ekf.Predict();

    auto predicted = ekf.GetState();
    // Position should increase slightly (initial velocity is 0, small position)
    // Velocity should become slightly negative (restoring force: -sin(0.1)*dt ≈ -0.01)
    EXPECT_NEAR(math::ToFloat(predicted.at(0, 0)), 0.1f, 0.05f);
    EXPECT_LT(math::ToFloat(predicted.at(1, 0)), 0.0f);
}

TYPED_TEST(ExtendedKalmanFilterTest, UpdateReducesCovariance)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    typename TestFixture::EkfType ekf(initialState, initialP,
        LinearStateTransition, LinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);

    ekf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    auto covBefore = ekf.GetCovariance();
    ekf.Update(typename TestFixture::MeasurementVector{ { TypeParam(0.5f) } });
    auto covAfter = ekf.GetCovariance();

    EXPECT_LT(math::ToFloat(covAfter.at(0, 0)), math::ToFloat(covBefore.at(0, 0)));
}

TYPED_TEST(ExtendedKalmanFilterTest, NonlinearTrackingConverges)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    typename TestFixture::EkfType ekf(initialState, initialP,
        NonlinearStateTransition, NonlinearStateJacobian,
        LinearMeasurement, LinearMeasurementJacobian);

    ekf.SetProcessNoise(StateMatrix{
        { TypeParam(0.001f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.001f) } });
    ekf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.1f) } });

    // Simulate a pendulum-like system
    float truePos = 0.3f;
    float trueVel = 0.0f;

    for (int i = 0; i < 50; ++i)
    {
        float newVel = trueVel - std::sin(truePos) * dt;
        float newPos = truePos + trueVel * dt;
        truePos = newPos;
        trueVel = newVel;

        ekf.Predict();
        ekf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ekf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.1f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.2f);
}

TYPED_TEST(ExtendedKalmanFilterTest, WithControlInput)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    typename TestFixture::EkfWithControlType ekf(initialState, initialP,
        StateTransitionWithControl, StateJacobianWithControl,
        LinearMeasurement, LinearMeasurementJacobian);

    ekf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });
    ekf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    // Apply constant acceleration
    ControlVec1 controlInput{ { 1.0f } };

    float truePos = 0.0f;
    float trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * dt;
        truePos += trueVel * dt;

        ekf.Predict(controlInput);
        ekf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ekf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.4f);
}
