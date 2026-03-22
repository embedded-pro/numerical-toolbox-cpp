#include "numerical/filters/active/UnscentedKalmanFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    using StateVec2 = math::Vector<float, 2>;
    using MeasVec1 = math::Vector<float, 1>;
    using StateMat2 = math::SquareMatrix<float, 2>;

    constexpr float dt = 0.1f;

    // Linear state transition for comparison with KF
    StateVec2 LinearStateTransition(const StateVec2& x)
    {
        return StateVec2{
            { float(x.at(0, 0) + dt * x.at(1, 0)) },
            { x.at(1, 0) }
        };
    }

    MeasVec1 LinearMeasurement(const StateVec2& x)
    {
        return MeasVec1{ { x.at(0, 0) } };
    }

    // Nonlinear pendulum-like system: position += velocity*dt, velocity += -sin(position)*dt
    StateVec2 NonlinearStateTransition(const StateVec2& x)
    {
        float pos = x.at(0, 0);
        float vel = x.at(1, 0);
        return StateVec2{
            { pos + vel * dt },
            { vel - std::sin(pos) * dt }
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

    template<typename T>
    class UnscentedKalmanFilterTest
        : public ::testing::Test
    {
    protected:
        float tolerance = math::Tolerance<T>();

        using UkfType = filters::UnscentedKalmanFilter<T, 2, 1, 0>;
        using UkfWithControlType = filters::UnscentedKalmanFilter<T, 2, 1, 1>;
        using StateVector = typename UkfType::StateVector;
        using StateMatrix = typename UkfType::StateMatrix;
        using MeasurementVector = typename UkfType::MeasurementVector;
        using MeasurementCovariance = typename UkfType::MeasurementCovariance;
    };

    using TestTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(UnscentedKalmanFilterTest, TestTypes);
}

TYPED_TEST(UnscentedKalmanFilterTest, LinearSystemTracksConstantVelocity)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    filters::UkfParameters params;
    params.alpha = 1e-1f;
    params.beta = 2.0f;
    params.kappa = 0.0f;

    typename TestFixture::UkfType ukf(initialState, initialP,
        LinearStateTransition, LinearMeasurement, params);

    ukf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });
    ukf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    float truePos = 0.0f;
    float trueVel = 0.5f;

    for (int i = 0; i < 10; ++i)
    {
        truePos += trueVel * dt;
        ukf.Predict();
        ukf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ukf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.5f);
}

TYPED_TEST(UnscentedKalmanFilterTest, PredictGrowsCovariance)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.1f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.1f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.1f) }
    };

    filters::UkfParameters params;
    params.alpha = 1e-1f;

    typename TestFixture::UkfType ukf(initialState, initialP,
        LinearStateTransition, LinearMeasurement, params);

    ukf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });

    auto covBefore = ukf.GetCovariance();
    ukf.Predict();
    auto covAfter = ukf.GetCovariance();

    EXPECT_GT(math::ToFloat(covAfter.at(0, 0)), math::ToFloat(covBefore.at(0, 0)));
}

TYPED_TEST(UnscentedKalmanFilterTest, UpdateReducesCovariance)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    filters::UkfParameters params;
    params.alpha = 1e-1f;

    typename TestFixture::UkfType ukf(initialState, initialP,
        LinearStateTransition, LinearMeasurement, params);

    ukf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    auto covBefore = ukf.GetCovariance();
    ukf.Update(typename TestFixture::MeasurementVector{ { TypeParam(0.5f) } });
    auto covAfter = ukf.GetCovariance();

    EXPECT_LT(math::ToFloat(covAfter.at(0, 0)), math::ToFloat(covBefore.at(0, 0)));
}

TYPED_TEST(UnscentedKalmanFilterTest, NonlinearTrackingConverges)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    filters::UkfParameters params;
    params.alpha = 1e-1f;
    params.beta = 2.0f;
    params.kappa = 0.0f;

    typename TestFixture::UkfType ukf(initialState, initialP,
        NonlinearStateTransition, LinearMeasurement, params);

    ukf.SetProcessNoise(StateMatrix{
        { TypeParam(0.001f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.001f) } });
    ukf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.1f) } });

    // Simulate a pendulum-like system
    float truePos = 0.3f;
    float trueVel = 0.0f;

    for (int i = 0; i < 50; ++i)
    {
        float newVel = trueVel - std::sin(truePos) * dt;
        float newPos = truePos + trueVel * dt;
        truePos = newPos;
        trueVel = newVel;

        ukf.Predict();
        ukf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ukf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.1f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.2f);
}

TYPED_TEST(UnscentedKalmanFilterTest, WithControlInput)
{
    using StateVector = typename TestFixture::StateVector;
    using StateMatrix = typename TestFixture::StateMatrix;
    using MeasurementCovariance = typename TestFixture::MeasurementCovariance;

    auto initialState = StateVector{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    auto initialP = StateMatrix{
        { TypeParam(0.5f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.5f) }
    };

    filters::UkfParameters params;
    params.alpha = 1e-1f;

    typename TestFixture::UkfWithControlType ukf(initialState, initialP,
        StateTransitionWithControl, LinearMeasurement, params);

    ukf.SetProcessNoise(StateMatrix{
        { TypeParam(0.01f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.01f) } });
    ukf.SetMeasurementNoise(MeasurementCovariance{ { TypeParam(0.5f) } });

    ControlVec1 controlInput{ { 1.0f } };

    float truePos = 0.0f;
    float trueVel = 0.0f;

    for (int i = 0; i < 10; ++i)
    {
        trueVel += 1.0f * dt;
        truePos += trueVel * dt;

        ukf.Predict(controlInput);
        ukf.Update(typename TestFixture::MeasurementVector{ { TypeParam(truePos) } });
    }

    auto finalState = ukf.GetState();
    EXPECT_NEAR(math::ToFloat(finalState.at(0, 0)), truePos, 0.2f);
    EXPECT_NEAR(math::ToFloat(finalState.at(1, 0)), trueVel, 0.5f);
}
