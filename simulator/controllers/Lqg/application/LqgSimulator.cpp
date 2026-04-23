#include "simulator/controllers/Lqg/application/LqgSimulator.hpp"
#include <cmath>
#include <random>

namespace simulator::controllers
{
    void LqgSimulator::Configure(const LqgPlant& newPlant, const LqgSimulatorConfig& config)
    {
        plant = newPlant;
        configuration = config;
    }

    LqgTimeResponse LqgSimulator::RunStepResponse()
    {
        LqgTimeResponse result;

        auto& w = configuration.weights;
        auto& n = configuration.noise;
        auto& sim = configuration.simulation;

        math::SquareMatrix<float, lqgStateSize> Q{
            { w.stateWeight, 0.0f },
            { 0.0f, w.stateWeight }
        };
        math::SquareMatrix<float, lqgInputSize> R{ { w.controlWeight } };

        math::SquareMatrix<float, lqgStateSize> processNoise{
            { n.processNoise, 0.0f },
            { 0.0f, n.processNoise }
        };
        math::SquareMatrix<float, lqgMeasurementSize> measurementNoise{ { n.measurementNoise } };

        math::Vector<float, lqgStateSize> initialState{};
        math::SquareMatrix<float, lqgStateSize> initialCovariance{
            { 1.0f, 0.0f },
            { 0.0f, 1.0f }
        };

        ::controllers::LqgNoise<float, lqgStateSize, lqgInputSize, lqgMeasurementSize> lqgNoise{
            processNoise, measurementNoise
        };
        ::controllers::LqgWeights<float, lqgStateSize, lqgInputSize> lqgWeights{ Q, R };

        LqgController controller(plant, lqgNoise, lqgWeights, initialState, initialCovariance);

        math::Vector<float, lqgStateSize> trueState{};
        trueState.at(0, 0) = configuration.initialPosition;

        std::mt19937 rng{ 42 };
        std::normal_distribution<float> processDist{ 0.0f, std::sqrt(n.processNoise) };
        std::normal_distribution<float> measureDist{ 0.0f, std::sqrt(n.measurementNoise) };

        std::size_t steps = static_cast<std::size_t>(sim.duration / sim.sampleTime);
        result.time.reserve(steps);
        result.trueState.reserve(steps);
        result.estimatedState.reserve(steps);
        result.control.reserve(steps);

        for (std::size_t k = 0; k < steps; ++k)
        {
            math::Vector<float, lqgMeasurementSize> measurement{};
            measurement.at(0, 0) = plant.Output(trueState, {}).at(0, 0) + measureDist(rng);

            auto u = controller.ComputeControl(measurement);

            math::Vector<float, lqgStateSize> noise{};
            noise.at(0, 0) = processDist(rng);
            noise.at(1, 0) = processDist(rng);

            trueState = plant.Step(trueState, u) + noise;

            result.time.push_back(static_cast<float>(k) * sim.sampleTime);
            result.trueState.push_back(trueState.at(0, 0));
            result.estimatedState.push_back(controller.GetEstimatedState().at(0, 0));
            result.control.push_back(u.at(0, 0));
        }

        return result;
    }
}
