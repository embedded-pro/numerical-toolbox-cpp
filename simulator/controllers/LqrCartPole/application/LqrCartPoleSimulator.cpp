#include "simulator/controllers/LqrCartPole/application/LqrCartPoleSimulator.hpp"
#include <algorithm>

namespace simulator::controllers::lqr
{
    LqrCartPoleSimulator::LqrCartPoleSimulator()
    {
        Configure(LqrCartPoleConfig{});
    }

    void LqrCartPoleSimulator::Configure(const LqrCartPoleConfig& config)
    {
        configuration = config;
        plant = CartPolePlant(config.plantParams);
        RecomputeGain();
    }

    void LqrCartPoleSimulator::Reset()
    {
        plant.Reset();
    }

    void LqrCartPoleSimulator::SetState(const CartPoleState& state)
    {
        plant.SetState(state);
    }

    void LqrCartPoleSimulator::RecomputeGain()
    {
        auto A = plant.LinearizedA(configuration.simulation.dt);
        auto B = plant.LinearizedB(configuration.simulation.dt);

        auto& w = configuration.weights;

        StateMatrix Q{
            { w.qX, 0.0f, 0.0f, 0.0f },
            { 0.0f, w.qXDot, 0.0f, 0.0f },
            { 0.0f, 0.0f, w.qTheta, 0.0f },
            { 0.0f, 0.0f, 0.0f, w.qThetaDot },
        };

        math::SquareMatrix<float, inputSize> R{ { w.rForce } };

        lqr = std::make_unique<::controllers::Lqr<float, stateSize, inputSize>>(A, B, Q, R);
    }

    float LqrCartPoleSimulator::ComputeControlForce() const
    {
        auto& s = plant.GetState();

        StateVector stateVec(s.x, s.xDot, s.theta, s.thetaDot);

        auto u = lqr->ComputeControl(stateVec);
        auto force = u.at(0, 0);

        return std::clamp(force, -configuration.simulation.forceLimit, configuration.simulation.forceLimit);
    }

    float LqrCartPoleSimulator::Step(float externalForce)
    {
        auto controlForce = ComputeControlForce();
        auto totalForce = controlForce + externalForce;
        auto clampedForce = std::clamp(totalForce, -configuration.simulation.forceLimit, configuration.simulation.forceLimit);

        plant.Step(clampedForce, configuration.simulation.dt);

        return controlForce;
    }

    const CartPoleState& LqrCartPoleSimulator::GetState() const
    {
        return plant.GetState();
    }

    const CartPoleParameters& LqrCartPoleSimulator::GetPlantParameters() const
    {
        return plant.GetParameters();
    }

    float LqrCartPoleSimulator::GetDt() const
    {
        return configuration.simulation.dt;
    }

    float LqrCartPoleSimulator::GetForceLimit() const
    {
        return configuration.simulation.forceLimit;
    }
}
