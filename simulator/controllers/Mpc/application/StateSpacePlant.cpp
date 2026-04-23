#include "simulator/controllers/Mpc/application/StateSpacePlant.hpp"
#include <cmath>

namespace simulator::controllers
{
    math::LinearTimeInvariant<float, 2, 1> MakeDoubleIntegrator(float dt)
    {
        return math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(
            math::SquareMatrix<float, 2>{
                { 1.0f, dt },
                { 0.0f, 1.0f } },
            math::Matrix<float, 2, 1>{
                { 0.5f * dt * dt },
                { dt } });
    }

    math::LinearTimeInvariant<float, 2, 1> MakeFirstOrderWithIntegrator(float gain, float timeConstant, float dt)
    {
        auto a = std::exp(-dt / timeConstant);
        auto b = gain * (1.0f - a);

        return math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(
            math::SquareMatrix<float, 2>{
                { 1.0f, dt },
                { 0.0f, a } },
            math::Matrix<float, 2, 1>{
                { 0.0f },
                { b } });
    }
}
