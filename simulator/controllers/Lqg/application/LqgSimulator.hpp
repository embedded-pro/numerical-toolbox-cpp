#pragma once

#include "numerical/controllers/implementations/Lqg.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include <vector>

namespace simulator::controllers
{
    static constexpr std::size_t lqgStateSize = 2;
    static constexpr std::size_t lqgInputSize = 1;
    static constexpr std::size_t lqgMeasurementSize = 1;

    using LqgPlant = math::LinearTimeInvariant<float, lqgStateSize, lqgInputSize, lqgMeasurementSize>;
    using LqgController = ::controllers::Lqg<float, lqgStateSize, lqgInputSize, lqgMeasurementSize>;

    struct LqgWeightsConfig
    {
        float stateWeight = 10.0f;
        float controlWeight = 0.1f;
    };

    struct LqgNoiseConfig
    {
        float processNoise = 0.1f;
        float measurementNoise = 1.0f;
    };

    struct LqgSimulationConfig
    {
        float duration = 10.0f;
        float sampleTime = 0.1f;
    };

    struct LqgSimulatorConfig
    {
        LqgWeightsConfig weights;
        LqgNoiseConfig noise;
        LqgSimulationConfig simulation;
        float initialPosition = 1.0f;
    };

    struct LqgTimeResponse
    {
        std::vector<float> time;
        std::vector<float> trueState;
        std::vector<float> estimatedState;
        std::vector<float> control;
    };

    class LqgSimulator
    {
    public:
        void Configure(const LqgPlant& plant, const LqgSimulatorConfig& config);
        LqgTimeResponse RunStepResponse();

    private:
        LqgPlant plant;
        LqgSimulatorConfig configuration;
    };
}
