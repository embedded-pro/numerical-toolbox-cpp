#include "simulator/controllers/Mpc/application/MpcSimulator.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include <cmath>

namespace simulator::controllers
{
    void MpcSimulator::Configure(const StateSpacePlant& plant, const Configuration& config)
    {
        this->plant = plant;
        configuration = config;
    }

    MpcTimeResponse MpcSimulator::ComputeStepResponse()
    {
        return Simulate(false);
    }

    MpcTimeResponse MpcSimulator::ComputeConstrainedResponse()
    {
        return Simulate(true);
    }

    MpcTimeResponse MpcSimulator::Simulate(bool enableConstraints)
    {
        auto dt = configuration.simulation.sampleTime;
        auto duration = configuration.simulation.duration;
        auto numSamples = static_cast<std::size_t>(duration / dt);

        auto A = math::SquareMatrix<float, 2>{
            { plant.A[0][0], plant.A[0][1] },
            { plant.A[1][0], plant.A[1][1] }
        };
        auto B = math::Matrix<float, 2, 1>{
            { plant.B[0][0] },
            { plant.B[1][0] }
        };

        auto weights = ::controllers::MpcWeights<float, 2, 1>{};
        weights.Q = math::SquareMatrix<float, 2>{
            { configuration.weights.stateWeight, 0.0f },
            { 0.0f, 1.0f }
        };
        weights.R = math::SquareMatrix<float, 1>{
            { configuration.weights.controlWeight }
        };

        auto constraints = ::controllers::MpcConstraints<float, 1>{};
        if (enableConstraints && configuration.constraints.enabled)
        {
            constraints.uMin = math::Vector<float, 1>{ { configuration.constraints.uMin } };
            constraints.uMax = math::Vector<float, 1>{ { configuration.constraints.uMax } };
        }

        auto mpc = ::controllers::Mpc<float, 2, 1, 10, 10>(A, B, weights, constraints);

        auto ref = math::Vector<float, 2>{ { configuration.referencePosition }, { 0.0f } };
        mpc.SetReference(ref);

        auto state = math::Vector<float, 2>{ { 0.0f }, { 0.0f } };

        auto response = MpcTimeResponse{};
        response.states.resize(2);

        for (std::size_t i = 0; i < numSamples; ++i)
        {
            response.time.push_back(static_cast<float>(i) * dt);
            response.states[0].push_back(state.at(0, 0));
            response.states[1].push_back(state.at(1, 0));

            auto u = mpc.ComputeControl(state);
            response.control.push_back(u.at(0, 0));

            auto posError = state.at(0, 0) - configuration.referencePosition;
            auto velError = state.at(1, 0);
            auto costStep = configuration.weights.stateWeight * posError * posError +
                            velError * velError +
                            configuration.weights.controlWeight * u.at(0, 0) * u.at(0, 0);
            response.cost.push_back(costStep);

            state = A * state + B * u;
        }

        return response;
    }
}
