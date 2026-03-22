#include "simulator/filters/KalmanFilter/application/PendulumPlant.hpp"

namespace simulator::filters
{
    PendulumPlant::PendulumPlant(PendulumParameters params)
        : parameters(params)
    {}

    PendulumState PendulumPlant::Step(float torque, float dt)
    {
        // θ'' = -(g/L)sin(θ) - b θ' + τ/(mL²)
        float g = parameters.gravity;
        float L = parameters.length;
        float b = parameters.damping;
        float m = parameters.mass;

        float thetaDotDot = -(g / L) * std::sin(state.theta) - b * state.thetaDot + torque / (m * L * L);

        // Semi-implicit Euler integration
        state.thetaDot += thetaDotDot * dt;
        state.theta += state.thetaDot * dt;

        return state;
    }

    void PendulumPlant::Reset(float theta0, float thetaDot0)
    {
        state.theta = theta0;
        state.thetaDot = thetaDot0;
    }

    PendulumState PendulumPlant::GetState() const
    {
        return state;
    }
}
