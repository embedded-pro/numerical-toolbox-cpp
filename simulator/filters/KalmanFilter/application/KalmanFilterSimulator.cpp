#include "simulator/filters/KalmanFilter/application/KalmanFilterSimulator.hpp"
#include "numerical/filters/active/ExtendedKalmanFilter.hpp"
#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/filters/active/UnscentedKalmanFilter.hpp"
#include <cmath>
#include <random>

namespace simulator::filters
{
    namespace
    {
        using StateVec = math::Vector<float, 2>;
        using StateMat = math::SquareMatrix<float, 2>;
        using MeasVec = math::Vector<float, 1>;
        using MeasCov = math::SquareMatrix<float, 1>;

        void ReserveResults(SimulationResult& result, std::size_t steps)
        {
            result.time.reserve(steps);
            result.trueTheta.reserve(steps);
            result.trueThetaDot.reserve(steps);
            result.measuredTheta.reserve(steps);

            auto reserveTrace = [steps](EstimatorTrace& trace)
            {
                trace.theta.reserve(steps);
                trace.thetaDot.reserve(steps);
                trace.covarianceTheta.reserve(steps);
            };

            reserveTrace(result.kf);
            reserveTrace(result.ekf);
            reserveTrace(result.ukf);
        }

        template<typename FilterType>
        void RecordFilterEstimate(EstimatorTrace& trace, FilterType& filter, const MeasVec& measurement)
        {
            filter.Predict();
            filter.Update(measurement);
            trace.theta.push_back(filter.GetState().at(0, 0));
            trace.thetaDot.push_back(filter.GetState().at(1, 0));
            trace.covarianceTheta.push_back(filter.GetCovariance().at(0, 0));
        }
    }

    SimulationResult KalmanFilterSimulator::Run(const SimulationConfig& config)
    {
        SimulationResult result;

        float g = config.pendulum.gravity;
        float L = config.pendulum.length;
        float b = config.pendulum.damping;
        float dt = config.dt;

        auto pendulumTransition = [g, L, b, dt](const StateVec& x) -> StateVec
        {
            float theta = x.at(0, 0);
            float thetaDot = x.at(1, 0);
            float thetaDotDot = -(g / L) * std::sin(theta) - b * thetaDot;
            return StateVec{
                { theta + thetaDot * dt },
                { thetaDot + thetaDotDot * dt }
            };
        };

        auto pendulumJacobian = [g, L, b, dt](const StateVec& x) -> StateMat
        {
            float theta = x.at(0, 0);
            return StateMat{
                { 1.0f, dt },
                { -(g / L) * std::cos(theta) * dt, 1.0f - b * dt }
            };
        };

        auto measurementFn = [](const StateVec& x) -> MeasVec
        {
            return MeasVec{ { x.at(0, 0) } };
        };

        auto measurementJacobian = [](const StateVec& /*x*/) -> math::Matrix<float, 1, 2>
        {
            return math::Matrix<float, 1, 2>{ { 1.0f, 0.0f } };
        };

        auto initialState = StateVec{ { config.initialTheta }, { config.initialThetaDot } };
        auto initialP = StateMat{
            { 0.5f, 0.0f },
            { 0.0f, 0.5f }
        };

        float qVar = config.processNoiseStdDev * config.processNoiseStdDev;
        auto Q = StateMat{
            { qVar, 0.0f },
            { 0.0f, qVar }
        };

        float rVar = config.measurementNoiseStdDev * config.measurementNoiseStdDev;
        auto R = MeasCov{ { rVar } };

        ::filters::KalmanFilter<float, 2, 1> kf(initialState, initialP);
        kf.SetStateTransition(StateMat{
            { 1.0f, dt },
            { -(g / L) * dt, 1.0f - b * dt } });
        kf.SetMeasurementMatrix(math::Matrix<float, 1, 2>{ { 1.0f, 0.0f } });
        kf.SetProcessNoise(Q);
        kf.SetMeasurementNoise(R);

        ::filters::ExtendedKalmanFilter<float, 2, 1> ekf(initialState, initialP,
            pendulumTransition, pendulumJacobian,
            measurementFn, measurementJacobian);
        ekf.SetProcessNoise(Q);
        ekf.SetMeasurementNoise(R);

        ::filters::UkfParameters ukfParams;
        ukfParams.alpha = 1e-1f;
        ukfParams.beta = 2.0f;
        ukfParams.kappa = 0.0f;

        ::filters::UnscentedKalmanFilter<float, 2, 1> ukf(initialState, initialP,
            pendulumTransition, measurementFn, ukfParams);
        ukf.SetProcessNoise(Q);
        ukf.SetMeasurementNoise(R);

        PendulumPlant plant(config.pendulum);
        plant.Reset(config.initialTheta, config.initialThetaDot);

        std::size_t steps = static_cast<std::size_t>(config.duration / config.dt);
        ReserveResults(result, steps);

        std::mt19937 rng(42);
        std::normal_distribution<float> measurementNoise(0.0f, config.measurementNoiseStdDev);
        std::normal_distribution<float> processNoise(0.0f, config.processNoiseStdDev);

        for (std::size_t i = 0; i < steps; ++i)
        {
            float t = static_cast<float>(i) * config.dt;

            float noisyTorque = processNoise(rng);
            auto trueState = plant.Step(noisyTorque, dt);
            float noisyTheta = trueState.theta + measurementNoise(rng);

            result.time.push_back(t);
            result.trueTheta.push_back(trueState.theta);
            result.trueThetaDot.push_back(trueState.thetaDot);
            result.measuredTheta.push_back(noisyTheta);

            auto measurement = MeasVec{ { noisyTheta } };

            RecordFilterEstimate(result.kf, kf, measurement);
            RecordFilterEstimate(result.ekf, ekf, measurement);
            RecordFilterEstimate(result.ukf, ukf, measurement);
        }

        return result;
    }
}
