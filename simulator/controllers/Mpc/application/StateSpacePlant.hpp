#pragma once

#include "numerical/math/LinearTimeInvariant.hpp"

namespace simulator::controllers
{
    math::LinearTimeInvariant<float, 2, 1> MakeDoubleIntegrator(float dt);
    math::LinearTimeInvariant<float, 2, 1> MakeFirstOrderWithIntegrator(float gain, float timeConstant, float dt);
}
