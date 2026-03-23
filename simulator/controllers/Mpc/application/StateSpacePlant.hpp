#pragma once

#include <vector>

namespace simulator::controllers
{
    struct StateSpacePlant
    {
        std::vector<std::vector<float>> A;
        std::vector<std::vector<float>> B;
        std::size_t stateSize;
        std::size_t inputSize;
    };

    StateSpacePlant MakeDoubleIntegrator(float dt);
    StateSpacePlant MakeFirstOrderWithIntegrator(float gain, float timeConstant, float dt);
}
