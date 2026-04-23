#pragma once

#include "numerical/math/LinearTimeInvariant.hpp"
#include <vector>

namespace simulator::controllers
{
    struct MpcSimulationConfig
    {
        float duration = 10.0f;
        float sampleTime = 0.1f;
    };

    struct MpcWeightsConfig
    {
        float stateWeight = 10.0f;
        float controlWeight = 0.1f;
    };

    struct MpcConstraintsConfig
    {
        bool enabled = false;
        float uMin = -1.0f;
        float uMax = 1.0f;
    };

    struct MpcTimeResponse
    {
        std::vector<float> time;
        std::vector<std::vector<float>> states;
        std::vector<float> control;
        std::vector<float> cost;
    };

    class MpcSimulator
    {
    public:
        struct Configuration
        {
            MpcWeightsConfig weights;
            MpcConstraintsConfig constraints;
            MpcSimulationConfig simulation;
            float referencePosition = 1.0f;
        };

        void Configure(const math::LinearTimeInvariant<float, 2, 1>& plant, const Configuration& config);
        MpcTimeResponse ComputeStepResponse();
        MpcTimeResponse ComputeConstrainedResponse();

    private:
        MpcTimeResponse Simulate(bool enableConstraints);

        math::LinearTimeInvariant<float, 2, 1> plant;
        Configuration configuration;
    };
}
