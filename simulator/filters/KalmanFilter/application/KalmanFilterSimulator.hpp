#pragma once

#include "simulator/filters/KalmanFilter/application/PendulumPlant.hpp"
#include <vector>

namespace simulator::filters
{
    struct SimulationConfig
    {
        float duration = 10.0f;
        float dt = 0.01f;
        float initialTheta = 0.5f;
        float initialThetaDot = 0.0f;
        float measurementNoiseStdDev = 0.1f;
        float processNoiseStdDev = 0.01f;
        PendulumParameters pendulum;
    };

    struct EstimatorTrace
    {
        std::vector<float> theta;
        std::vector<float> thetaDot;
        std::vector<float> covarianceTheta;
    };

    struct SimulationResult
    {
        std::vector<float> time;
        std::vector<float> trueTheta;
        std::vector<float> trueThetaDot;
        std::vector<float> measuredTheta;
        EstimatorTrace kf;
        EstimatorTrace ekf;
        EstimatorTrace ukf;
    };

    class KalmanFilterSimulator
    {
    public:
        SimulationResult Run(const SimulationConfig& config);
    };
}
