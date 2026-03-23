#include "simulator/controllers/Mpc/application/StateSpacePlant.hpp"

namespace simulator::controllers
{
    StateSpacePlant MakeDoubleIntegrator(float dt)
    {
        auto plant = StateSpacePlant{};
        plant.stateSize = 2;
        plant.inputSize = 1;
        plant.A = {
            { 1.0f, dt },
            { 0.0f, 1.0f }
        };
        plant.B = {
            { 0.5f * dt * dt },
            { dt }
        };
        return plant;
    }

    StateSpacePlant MakeFirstOrderWithIntegrator(float gain, float timeConstant, float dt)
    {
        auto a = std::exp(-dt / timeConstant);
        auto b = gain * (1.0f - a);

        auto plant = StateSpacePlant{};
        plant.stateSize = 2;
        plant.inputSize = 1;
        plant.A = {
            { 1.0f, dt },
            { 0.0f, a }
        };
        plant.B = {
            { 0.0f },
            { b }
        };
        return plant;
    }
}
