#include "simulator/controllers/Mpc/application/StateSpacePlant.hpp"
#include <cmath>

namespace simulator::controllers
{
    StateSpacePlant MakeDoubleIntegrator(float dt)
    {
        return StateSpacePlant{
            .A = { { 1.0f, dt }, { 0.0f, 1.0f } },
            .B = { { 0.5f * dt * dt }, { dt } },
            .stateSize = 2,
            .inputSize = 1
        };
    }

    StateSpacePlant MakeFirstOrderWithIntegrator(float gain, float timeConstant, float dt)
    {
        auto a = std::exp(-dt / timeConstant);
        auto b = gain * (1.0f - a);

        return StateSpacePlant{
            .A = { { 1.0f, dt }, { 0.0f, a } },
            .B = { { 0.0f }, { b } },
            .stateSize = 2,
            .inputSize = 1
        };
    }
}
